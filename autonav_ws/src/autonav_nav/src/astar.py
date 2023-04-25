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
cost_map = None
best_pos = (0, 0)
waypoints = []
first_waypoints_time = -1
verticalCameraRange = 2.75
horizontalCameraRange = 3

MAP_RES = 80

orig_waypoints = [(42.66792771,-83.21932764),(42.66807663,-83.21935916),(42.66826972,-83.21934030)]
sim_waypoints = [(35.19487762, -97.43902588), (35.19476700, -97.43901825), (35.19472504, -97.43901825)]
# sim_waypoints = []


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
        first_waypoints_time = time.time()
        self.m_mapTimer = self.create_timer(0.3, self.makeMap)
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
        pass

    def onPoseReceived(self, msg: Position):
        self.m_position = msg
        
    def makeMap(self):
        global cost_map, best_pos, planner
        if self.m_position is None or cost_map is None:
            return
        
        robot_pos = (MAP_RES // 2, MAP_RES - 4)
        path = self.find_path_to_point(robot_pos, best_pos, cost_map, MAP_RES, MAP_RES)
        
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
        looked_at = np.zeros((MAP_RES, MAP_RES))
        open_set = [start]
        path = {}
        search_dirs = []

        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == 0 and y == 0:
                    continue
                search_dirs.append((x,y,math.sqrt(x ** 2 + y ** 2)))

        def h(point):
            return math.sqrt((goal[0] - point[0]) ** 2 + (goal[1] - point[1]) ** 2)

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
        global cost_map, best_pos, planner, first_waypoints_time, waypoints
        
        if self.m_position is None:
            return

        grid_data = msg.data

        temp_best_pos = (MAP_RES // 2, MAP_RES - 4)
        best_pos_cost = -1000
        frontier = set()
        frontier.add((MAP_RES // 2, MAP_RES - 4))
        explored = set()

        if first_waypoints_time > 0 and time.time() > first_waypoints_time:
            next_waypoint = [pt for pt in (sim_waypoints if self.getSystemState().is_simulator else orig_waypoints)][0]
            north_to_gps = (next_waypoint[0] - self.m_position.latitude) * 111086.2
            west_to_gps = (self.m_position.longitude - next_waypoint[1]) * 81978.2
            dist = math.sqrt(north_to_gps ** 2 + west_to_gps ** 2)
            if dist < 100: # Only apply the waypoint costs if we are near no mans land
                first_waypoints_time = -2
                waypoints = [pt for pt in (sim_waypoints if self.getSystemState().is_simulator else orig_waypoints)]

        if len(waypoints) > 0:
            next_waypoint = waypoints[0]
            north_to_gps = (next_waypoint[0] - self.m_position.latitude) * 111086.2
            west_to_gps = (self.m_position.longitude - next_waypoint[1]) * 81978.2
            heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi)

            if north_to_gps ** 2 + west_to_gps ** 2 <= 1:
                waypoints.pop(0)

        depth = 0
        while depth < 50 and len(frontier) > 0:
            curfrontier = copy.copy(frontier)
            for pos in curfrontier:
                x = pos[0]
                y = pos[1]
                cost = (MAP_RES - y) * 1.3 + depth * 2.2

                if len(waypoints) > 0:
                    heading_err_to_gps = abs(get_angle_diff(self.m_position.theta + math.atan2(MAP_RES // 2- x, MAP_RES - y), heading_to_gps)) * 180 / math.pi
                    cost -= max(heading_err_to_gps, 10)

                if cost > best_pos_cost:
                    best_pos_cost = cost
                    temp_best_pos = pos

                frontier.remove(pos)
                explored.add(x + MAP_RES * y)

                if y > 1 and grid_data[x + MAP_RES * (y-1)] < 50 and x + MAP_RES * (y-1) not in explored:
                    frontier.add((x, y-1))

                if x < 79 and grid_data[x + 1 + MAP_RES * y] < 50 and x + 1 + MAP_RES * y not in explored:
                    frontier.add((x+1, y))
                if x > 0 and grid_data[x - 1 + MAP_RES * y] < 50 and x - 1 + MAP_RES * y not in explored:
                    frontier.add((x-1, y))

            depth += 1

        cost_map = grid_data
        best_pos = temp_best_pos
        map_init = False
        
def pathToGlobalPose(robot, pp0, pp1):
    x = (MAP_RES - pp1) * verticalCameraRange / MAP_RES
    y = (MAP_RES // 2 - pp0) * horizontalCameraRange / MAP_RES
    dx = 0
    dy = 0
    psi = 0
    
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
