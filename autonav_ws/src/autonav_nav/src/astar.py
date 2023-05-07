#!/usr/bin/env python3

from autonav_msgs.msg import Position
from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum
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
verticalCameraRange = 2.75
horizontalCameraRange = 3

MAP_RES = 80
CONFIG_WAYPOINT_POP_DISTANCE = 0, # The distance until the waypoint has been reached
CONFIG_WAYPOINT_DIRECTION = 1 # 0 for northbound, 1 for southbound, 2 for misc
CONFIG_WAYPOINT_ACTIVATION_DISTANCE = 2 # The distance from the robot to the next waypoint that will cause the robot to start using waypoints

### Waypoints for navigation, the first index is northbound waypoints, the second index is southbound waypoints, a 3rd index can be added for misc waypoints
# Practice Waypoints: (42.668222,-83.218472),(42.6680859611,-83.2184456444),(42.6679600583,-83.2184326556) | North, Mid, South
# AutoNav Waypoints: (42.6682697222,-83.2193403028),(42.6681206444,-83.2193606083),(42.6680766333,-83.2193591583),(42.6679277056,-83.2193276417) | North, North Ramp, South Ramp, South
## The distance from north to north ramp is about 16.6 meters
## The ramp is about 5m
## The distance from south ramp to south is about 16.76 meters
## The total distance from north to south is about 38.045 meters

competition_waypoints = [
    [(42.6682697222,-83.2193403028),(42.6681206444,-83.2193606083),(42.6680766333,-83.2193591583),(42.6679277056,-83.2193276417)], 
    [(42.6679277056,-83.2193276417),(42.6680766333,-83.2193591583),(42.6681206444,-83.2193606083),(42.6682697222,-83.2193403028)], 
    []
]
# competition_waypoints = [[], [], []]

practice_waypoints = [
    [(42.668222,-83.218472),(42.6680859611,-83.2184456444),(42.6679600583,-83.2184326556)],
    [(42.6679600583,-83.2184326556),(42.6680859611,-83.2184456444),(42.668222,-83.218472)],
    []
]
practice_waypoints = [[], [], []]

simulation_waypoints = [
    [(35.19478989, -97.43856812), (35.19480515, -97.43852997), (35.19487762, -97.43852997)], 
    [(35.19478989, -97.43856812), (35.19480515, -97.43852997), (35.19487762, -97.43852997)], 
    []
]
simulation_waypoints = [[], [], []]

def get_angle_diff(to_angle, from_angle):
    delta = to_angle - from_angle
    delta = (delta + math.pi) % (2 * math.pi) - math.pi
    return delta


class AStarNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_astar")

        self.m_configSpace = None
        self.position = None

    def configure(self):
        self.config.setFloat(CONFIG_WAYPOINT_POP_DISTANCE, 1.0)
        self.config.setInt(CONFIG_WAYPOINT_DIRECTION, 0)
        self.config.setFloat(CONFIG_WAYPOINT_ACTIVATION_DISTANCE, 5)

        self.m_configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.onConfigSpaceReceived, 20)
        self.m_poseSubscriber = self.create_subscription(Position, "/autonav/position", self.onPoseReceived, 20)
        self.m_pathPublisher = self.create_publisher(Path, "/autonav/path", 20)
        self.m_mapTimer = self.create_timer(0.3, self.makeMap)
        self.setDeviceState(DeviceStateEnum.READY)
        
    def onReset(self):
        global cost_map, best_pos, waypoints
        self.position = None
        self.m_configSpace = None
        cost_map = None
        best_pos = (0, 0)
        waypoints = []

    def getWaypointsForDirection(self):
        direction_index = self.config.getInt(CONFIG_WAYPOINT_DIRECTION)
        return simulation_waypoints[direction_index] if self.getSystemState().is_simulator else competition_waypoints[direction_index]

    def transition(self, old: SystemState, updated: SystemState):
        global waypoints
        if updated.state == SystemStateEnum.AUTONOMOUS and self.getDeviceState() == DeviceStateEnum.READY:
            self.setDeviceState(DeviceStateEnum.OPERATING)
            
        if updated.state != SystemStateEnum.AUTONOMOUS and self.getDeviceState() == DeviceStateEnum.OPERATING:
            self.setDeviceState(DeviceStateEnum.READY)
            
    def onPoseReceived(self, msg: Position):
        self.position = msg
        
    def makeMap(self):
        global cost_map, best_pos, planner
        if self.position is None or cost_map is None:
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

        self.performance.end("A*")
        return total_path[::-1]
        
    def find_path_to_point(self, start, goal, map, width, height):
        looked_at = np.zeros((MAP_RES, MAP_RES))
        open_set = [start]
        path = {}
        search_dirs = []

        self.performance.start("A*")

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
        global cost_map, best_pos, planner, waypoints
        if self.position is None or self.getDeviceState() != DeviceStateEnum.OPERATING or self.getSystemState().state != SystemStateEnum.AUTONOMOUS:
            return

        self.performance.start("Smellification")

        grid_data = msg.data
        temp_best_pos = (MAP_RES // 2, MAP_RES - 4)
        best_pos_cost = -1000
        frontier = set()
        frontier.add((MAP_RES // 2, MAP_RES - 4))
        explored = set()

        grid_data = [0] * len(msg.data)

        if len(waypoints) == 0:
            wpt = self.getWaypointsForDirection()[0]
            north_to_gps = (wpt[0] - self.position.latitude) * 111086.2
            west_to_gps = (self.position.longitude - wpt[1]) * 81978.2
            distanceToWaypoint = math.sqrt(north_to_gps ** 2 + west_to_gps ** 2)
            if distanceToWaypoint <= self.config.getFloat(CONFIG_WAYPOINT_ACTIVATION_DISTANCE):
                waypoints = self.getWaypointsForDirection()

        if len(waypoints) > 0:
            next_waypoint = waypoints[0]
            north_to_gps = (next_waypoint[0] - self.position.latitude) * 111086.2
            west_to_gps = (self.position.longitude - next_waypoint[1]) * 81978.2
            heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi)

            self.log("Heading to GPS: " + str(heading_to_gps * 180 / math.pi) + " | (" + str(north_to_gps) + ", " + str(west_to_gps) + ")")
            if north_to_gps ** 2 + west_to_gps ** 2 <= self.config.getFloat(CONFIG_WAYPOINT_POP_DISTANCE):
                waypoints.pop(0)

        depth = 0
        while depth < 50 and len(frontier) > 0:
            curfrontier = copy.copy(frontier)
            for pos in curfrontier:
                x = pos[0]
                y = pos[1]
                cost = (MAP_RES - y) * 1.3 + depth * 2.2

                if len(waypoints) > 0:
                    heading_err_to_gps = abs(get_angle_diff(self.position.theta + math.atan2(MAP_RES // 2 - x, MAP_RES - y), heading_to_gps)) * 180 / math.pi
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

        self.performance.end("Smellification")
        
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
