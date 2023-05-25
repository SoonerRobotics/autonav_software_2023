#!/usr/bin/env python3

from autonav_msgs.msg import Position, IMUData, PathingDebug
from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum, SystemMode
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
import math
import copy
from heapq import heappush, heappop
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import cv_bridge


GRID_SIZE = 0.1
cost_map = None
best_pos = (0, 0)
waypoints = []
verticalCameraRange = 2.75
horizontalCameraRange = 3

bridge = cv_bridge.CvBridge()

MAP_RES = 80
CONFIG_WAYPOINT_POP_DISTANCE = "pop_distance" # The distance until the waypoint has been reached
CONFIG_WAYPOINT_DIRECTION = "direction" # 0 for northbound, 1 for southbound, 2 for misc1, 3 for misc2, 4 for misc3, 5 for misc4, and 6 for misc5
CONFIG_USE_ONLY_WAYPOINTS = "use_only_waypoints" # 0 for no waypoints, 1 for waypoints
CONFIG_USE_IMU_HEADING = "use_imu_heading"
CONFIG_WAYPOINT_ACTIVATION_DISTANCE = "activation_distance" # The distance until the first waypoint is activated
CONFIG_NORMALIZE_WAYPOINT_DISTANCES = "normalize_waypoint_distances" # 0 for no normalization, 1 for normalization

### Waypoints for navigation, the first index is northbound waypoints, the second index is southbound waypoints, a 3rd index can be added for misc waypoints
# Practice Waypoints: (42.668222,-83.218472),(42.6680859611,-83.2184456444),(42.6679600583,-83.2184326556) | North, Mid, South
# AutoNav Waypoints: (42.6682697222,-83.2193403028),(42.6681206444,-83.2193606083),(42.6680766333,-83.2193591583),(42.6679277056,-83.2193276417) | North, North Ramp, South Ramp, South
## The distance from north to north ramp is about 16.6 meters
## The ramp is about 5m
## The distance from south ramp to south is about 16.76 meters
## The total distance from north to south is about 38.045 meters

competition_waypoints = [
    [(42.6682697222,-83.2193403028),(42.6681206444,-83.2193606083),(42.6680766333,-83.2193591583),(42.6679277056,-83.2193276417)], 
    [(42.6679277056,-83.2193276417),(42.6680766333,-83.2193591583),(42.6681206444,-83.2193606083),(42.6682697222,-83.2193403028)]
]
# competition_waypoints = [[], [], []]

practice_waypoints = [
    [(35.2104852, -97.44193), (35.2104884, -97.4421088), (35.2104819, -97.4423302), (35.2106063, -97.4423293), (35.2106045, -97.4421059), (35.2104852, -97.44193)],
    [(42.668222,-83.218472),(42.6680859611,-83.2184456444),(42.6679600583,-83.2184326556)],
    [(42.6679600583,-83.2184326556),(42.6680859611,-83.2184456444),(42.668222,-83.218472)],
]
# practice_waypoints = [[], [], []]

simulation_waypoints = [
    [(35.19496918, -97.43855286), (35.19475331, -97.43860863), (35.19493775, -97.43881094)], 
    [(35.19496918, -97.43855286), (35.19498780, -97.43859524), (35.19495163, -97.43871339), (35.19495182, -97.43877829), (35.19493406, -97.43879432), (35.19493812, -97.43881810), (35.19495368, -97.43882411), (35.19493357, -97.43902773), (35.19485847, -97.43902381)], 
    [(35.19474411, -97.43852997), (35.19474792, -97.43863678), (35.19484711, -97.43865967), (35.19494247, -97.43875885)]
]
# simulation_waypoints = [[], [], []]

def get_angle_diff(to_angle, from_angle):
    delta = to_angle - from_angle
    delta = (delta + math.pi) % (2 * math.pi) - math.pi
    return delta


class AStarNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_astar")

        self.configSpace = None
        self.position = None
        self.imu = None
        self.lastPath = None
        
        self.latitudeLength = self.declare_parameter("latitude_length", 111086.2).get_parameter_value().double_value
        self.longitudeLength = self.declare_parameter("longitude_length", 81978.2).get_parameter_value().double_value

    def configure(self):
        self.configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.onConfigSpaceReceived, 20)
        self.poseSubscriber = self.create_subscription(Position, "/autonav/position", self.onPoseReceived, 20)
        self.imuSubscriber = self.create_subscription(IMUData, "/autonav/imu", self.onImuReceived, 20)
        self.debugPublisher = self.create_publisher(PathingDebug, "/autonav/debug/astar", 20)
        self.pathPublisher = self.create_publisher(Path, "/autonav/path", 20)
        self.rawConfigSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/raw", self.onRawConfigSpaceReceived, 20)
        self.pathDebugImagePublisher = self.create_publisher(CompressedImage, "/autonav/debug/astar/image", 20)
        self.mapTimer = self.create_timer(0.2, self.makeMap)

        self.config.setFloat(CONFIG_WAYPOINT_POP_DISTANCE, 1.0)
        self.config.setInt(CONFIG_WAYPOINT_DIRECTION, 0)
        self.config.setBool(CONFIG_USE_ONLY_WAYPOINTS, True)
        self.config.setBool(CONFIG_USE_IMU_HEADING, False)
        self.config.setFloat(CONFIG_WAYPOINT_ACTIVATION_DISTANCE, 5.0)
        self.config.setBool(CONFIG_NORMALIZE_WAYPOINT_DISTANCES, False)

        self.setDeviceState(DeviceStateEnum.READY)

    def onImuReceived(self, msg: IMUData):
        self.imu = msg
        
    def onReset(self):
        global cost_map, best_pos, waypoints
        self.position = None
        self.configSpace = None
        cost_map = None
        best_pos = (0, 0)
        waypoints = []
        
    def onRawConfigSpaceReceived(self, msg: OccupancyGrid):
        if self.lastPath is None:
            return
        
        mat = np.array(msg.data, dtype=np.uint8).reshape((80, 80))
        mat = cv2.cvtColor(mat, cv2.COLOR_GRAY2BGR)
        
        for i in range(len(self.lastPath) - 1):
            cv2.line(mat, (self.lastPath[i][0], self.lastPath[i][1]), (self.lastPath[i + 1][0], self.lastPath[i + 1][1]), (0, 0, 255), 2)
            
        # Resize to 800x800
        mat = cv2.resize(mat, (800, 800), interpolation=cv2.INTER_NEAREST)
            
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', mat)[1]).tobytes()
        self.pathDebugImagePublisher.publish(msg)
    

    def getWaypointsForDirection(self):
        direction_index = self.config.getInt(CONFIG_WAYPOINT_DIRECTION)
        wpts = simulation_waypoints[direction_index] if self.getSystemState().mode == SystemMode.SIMULATION else competition_waypoints[direction_index] if self.getSystemState().mode == SystemMode.COMPETITION else practice_waypoints[direction_index]
        return self.normalizeWaypoints(wpts) if self.config.getBool(CONFIG_NORMALIZE_WAYPOINT_DISTANCES) else wpts

    def normalizeWaypoints(self, waypoints):
        # Ensure that the distance between any two waypoint is <= 25.0, if not add points in between
        new_waypoints = []
        for i in range(len(waypoints) - 1):
            new_waypoints.append(waypoints[i])
            wpt1 = waypoints[i]
            wpt2 = waypoints[i + 1]
            north_to_gps = (wpt1[0] - wpt2[0]) * self.latitudeLength
            west_to_gps = (wpt2[1] - wpt1[1]) * self.longitudeLength
            distanceToWaypoint = math.sqrt(north_to_gps ** 2 + west_to_gps ** 2)
            if distanceToWaypoint > 5.0:
                num_points = int(distanceToWaypoint / 5.0)
                for j in range(num_points):
                    new_waypoints.append((wpt1[0] + (wpt2[0] - wpt1[0]) * (j + 1) / (num_points + 1), wpt1[1] + (wpt2[1] - wpt1[1]) * (j + 1) / (num_points + 1)))

        new_waypoints.append(waypoints[-1])
        return new_waypoints

    def transition(self, old: SystemState, updated: SystemState):
        global waypoints
        if updated.state == SystemStateEnum.AUTONOMOUS and self.getDeviceState() == DeviceStateEnum.READY:
            self.setDeviceState(DeviceStateEnum.OPERATING)
            
        if updated.state != SystemStateEnum.AUTONOMOUS and self.getDeviceState() == DeviceStateEnum.OPERATING:
            waypoints = []
            self.setDeviceState(DeviceStateEnum.READY)
            
    def onPoseReceived(self, msg: Position):
        self.position = msg
        if self.config.getBool(CONFIG_USE_IMU_HEADING):
            good_yaw = (self.imu.yaw if self.imu.yaw > 0 else self.imu.yaw + 360) if self.imu is not None else self.position.theta
            self.position.theta = good_yaw
        
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
            self.lastPath = path
            self.pathPublisher.publish(global_path)
            
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

        if self.config.getBool(CONFIG_USE_ONLY_WAYPOINTS) == True:
            grid_data = [0] * len(msg.data)
            
        if len(waypoints) == 0:
            wpts = self.getWaypointsForDirection()
            if len(wpts) > 0:
                wpt = wpts[0]
                north_to_gps = (wpt[0] - self.position.latitude) * self.latitudeLength
                west_to_gps = (self.position.longitude - wpt[1]) * self.longitudeLength
                distanceToWaypoint = math.sqrt(north_to_gps ** 2 + west_to_gps ** 2)
                if distanceToWaypoint <= self.config.getFloat(CONFIG_WAYPOINT_ACTIVATION_DISTANCE):
                    waypoints = wpts
                
                pathingDebug = PathingDebug()
                pathingDebug.desired_heading = 0.0
                pathingDebug.desired_latitude = wpt[0]
                pathingDebug.desired_longitude = wpt[1]
                pathingDebug.distance_to_destination = distanceToWaypoint
                pathingDebug.waypoints = []
                self.debugPublisher.publish(pathingDebug)

        if len(waypoints) > 0:
            next_waypoint = waypoints[0]
            north_to_gps = (next_waypoint[0] - self.position.latitude) * self.latitudeLength
            west_to_gps = (self.position.longitude - next_waypoint[1]) * self.longitudeLength
            heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi)

            if math.sqrt(north_to_gps ** 2 + west_to_gps ** 2) <= self.config.getFloat(CONFIG_WAYPOINT_POP_DISTANCE):
                waypoints.pop(0)
                self.log("Reached waypoint, %d remaining" % len(waypoints))

            pathingDebug = PathingDebug()
            pathingDebug.desired_heading = heading_to_gps
            pathingDebug.desired_latitude = next_waypoint[0]
            pathingDebug.desired_longitude = next_waypoint[1]
            pathingDebug.distance_to_destination = math.sqrt(north_to_gps ** 2 + west_to_gps ** 2)
            wp1d = []
            for wp in waypoints:
                wp1d.append(wp[0])
                wp1d.append(wp[1])
            pathingDebug.waypoints = wp1d
            self.debugPublisher.publish(pathingDebug)

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
