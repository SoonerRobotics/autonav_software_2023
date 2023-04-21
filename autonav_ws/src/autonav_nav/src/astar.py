#!/usr/bin/env python3

from autonav_msgs.msg import Position
from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
import math
import copy
from heapq import heappush, heappop
import numpy as np
import time


GRID_SIZE = 0.1
prev_state = (0, 0)
map_ref = (0, 0, 0)
path_seq = 0
cost_map = None
best_pos = (0, 0)
waypoints = []
first_waypoints_time = -1
verticalCameraRange = 2.75
horizontalCameraRange = 3

orig_waypoints = [(42.66792771,-83.21932764),(42.66807663,-83.21935916),(42.66826972,-83.21934030)]
# sim_waypoints = [(35.19495300, -97.4385070), (35.19493840, -97.4384430)]
sim_waypoints = []


def get_angle_diff(to_angle, from_angle):
    delta = to_angle - from_angle
    delta = (delta + math.pi) % (2 * math.pi) - math.pi
    return delta


class AStarNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_astar")

        self.m_configSpace = None
        self.m_position = None

    def configure(self):
        global first_waypoints_time
        self.m_configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.onConfigSpaceReceived, 20)
        self.m_poseSubscriber = self.create_subscription(Position, "/autonav/position", self.onPoseReceived, 20)
        self.m_pathPublisher = self.create_publisher(Path, "/autonav/path", 20)
        self.m_mapTimer = self.create_timer(0.3, self.makeMap)
        
        first_waypoints_time = time.time()
        self.setDeviceState(DeviceStateEnum.OPERATING)

    def onReset(self):
        global first_waypoints_time, cost_map, best_pos, waypoints
        self.m_position = None
        self.m_configSpace = None
        first_waypoints_time = time.time()
        cost_map = None
        best_pos = (0, 0)
        waypoints = []

    def transition(self, _: SystemState, updated: SystemState):
        return

    def onPoseReceived(self, msg: Position):
        self.m_position = msg
        
    def makeMap(self):
        global cost_map, best_pos, planner, prev_state, map_ref, path_seq
        if self.m_position is None or cost_map is None:
            return
        
        robot_pos = (40, 78)
        path = self.find_path_to_point(robot_pos, best_pos, cost_map, 80, 80)
        
        if path is not None:
            global_path = Path()
            global_path.poses = [
                pathToGlobalPose(robot_pos, pp[0], pp[1]) for pp in path
            ]
            self.m_pathPublisher.publish(global_path)
            
    def reconstruct_path(self, path, current):
        total_path = [current]

        while current in path:
            current = path[current]
            total_path.append(current)

        return total_path[::-1]
        
    def find_path_to_point(self, start, goal, map, width, height):
        looked_at = np.zeros((80, 80))
        

        open_set = [start]

        path = {}

        search_dirs = []

        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == 0 and y == 0:
                    continue
                search_dirs.append((x,y,math.sqrt(x**2+y**2)))

        # print(search_dirs)

        def h(point):
            return math.sqrt((goal[0] - point[0])**2 + (goal[1] - point[1])**2)

        # assumes adjacent pts
        def d(to_pt, dist):
            return dist + map[to_pt[1] * width + to_pt[0]] / 10

        gScore = {}
        gScore[start] = 0

        def getG(pt):
            if pt in gScore:
                return gScore[pt]
            else:
                gScore[pt] = 1000000000
                return 1000000000 # Infinity

        fScore = {}
        fScore[start] = h(start)

        def getF(pt):
            if pt in fScore:
                return fScore[pt]
            else:
                fScore[pt] = 1000000000
                return 1000000000 # Infinity

        next_current = [(1,start)]
        while len(open_set) != 0:
            current = heappop(next_current)[1]

            looked_at[current[0],current[1]] = 1

            if current == goal:
                return self.reconstruct_path(path, current)

            open_set.remove(current)
            for delta_x, delta_y, dist in search_dirs:

                neighbor = (current[0] + delta_x, current[1] + delta_y)
                if neighbor[0] < 0 or neighbor[0] >= width or neighbor[1] < 0 or neighbor[1] >= height:
                    continue

                tentGScore = getG(current) + d(neighbor, dist)
                if tentGScore < getG(neighbor):
                    path[neighbor] = current
                    gScore[neighbor] = tentGScore
                    fScore[neighbor] = tentGScore + h(neighbor)
                    if neighbor not in open_set:
                        open_set.append(neighbor)
                        heappush(next_current, (fScore[neighbor], neighbor))
                    
    def onConfigSpaceReceived(self, msg: OccupancyGrid):
        global cost_map, best_pos, planner, prev_state, map_ref, path_seq, first_waypoints_time, waypoints
        
        if self.m_position is None:
            return

        grid_data = msg.data

        # Find the best position
        temp_best_pos = (40, 78)
        best_pos_cost = -1000
        frontier = set()
        frontier.add((40,78))
        explored = set()

        if first_waypoints_time > 0 and time.time() > first_waypoints_time:
            first_waypoints_time = -2
            waypoints = [pt for pt in (sim_waypoints if self.getSystemState().is_simulator else orig_waypoints)]

        if len(waypoints) > 0:
            next_waypoint = waypoints[0]
            north_to_gps = (next_waypoint[0] - self.m_position.latitude) * 111086.2
            west_to_gps = (self.m_position.longitude - next_waypoint[1]) * 81978.2
            heading_to_gps = math.atan2(west_to_gps,north_to_gps) % (2 * math.pi)

            if north_to_gps**2 + west_to_gps**2 <= 1:
                # mobi_start_publisher.publish(Bool(False))
                waypoints.pop(0)

        # sys.stdout.flush()
        # best_heading_err = 0

        depth = 0
        while depth < 50 and len(frontier) > 0:
            curfrontier = copy.copy(frontier)
            for pos in curfrontier:
                x = pos[0] # left to right
                y = pos[1] # top to botom
                # Cost at a point is sum of
                # - Negative X value (encourage forward)
                # - Positive Y value (discourage left/right)
                # - Heading
                cost = (80 - y) * 1.3 + depth * 2.2

                if len(waypoints) > 0:
                    heading_err_to_gps = abs(get_angle_diff(self.m_position.theta + math.atan2(40-x,80-y), heading_to_gps)) * 180 / math.pi
                    cost -= max(heading_err_to_gps, 10)

                if cost > best_pos_cost:
                    best_pos_cost = cost
                    temp_best_pos = pos
                    # best_heading_err = heading_err_to_gps

                frontier.remove(pos)
                explored.add(x + 80 * y)

                # Look left/right for good points
                if y > 1 and grid_data[x + 80 * (y-1)] < 50 and x + 80 * (y-1) not in explored:
                    frontier.add((x, y-1))

                # Look forward/back for good points
                if x < 79 and grid_data[x + 1 + 80 * y] < 50 and x + 1 + 80 * y not in explored:
                    frontier.add((x+1, y))
                if x > 0 and grid_data[x - 1 + 80 * y] < 50 and x - 1 + 80 * y not in explored:
                    frontier.add((x-1, y))

            depth += 1

        # print(f"best_heading_err: {best_heading_err:0.01f}")

        # map_reference = (curEKF.x, curEKF.y, curEKF.yaw)
        map_reference = (0,0,0)
        cost_map = grid_data
        best_pos = temp_best_pos
        map_init = False
        
def pathToGlobalPose(robot, pp0, pp1):
    x = (80 - pp1) * verticalCameraRange / 80
    y = (40 - pp0) * horizontalCameraRange / 80
    dx = map_ref[0]
    dy = map_ref[1]
    psi = map_ref[2]
    
    new_x = x * math.cos(psi) + y * math.sin(psi) + dx
    new_y = x * math.sin(psi) + y * math.cos(psi) + dy
    pose = PoseStamped()
    point = Point()
    point.x = new_x
    point.y = new_y
    pose.pose.position = point
    return pose

def main():
    rclpy.init()
    rclpy.spin(AStarNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
