#!/usr/bin/env python3

import tangent_based
import planning_test
import random
import math
import time
import rclpy
from rclpy.node import Node
from autonav_msgs.msg import Waypoint, Path
from autonav_msgs.msg import Obstacle, Obstacles
from autonav_msgs.msg import Position

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planning")
        self.publisher = self.create_publisher(Path, '/autonav/Path', 10)
        self.subscriber = self.create_subscription(Obstacles, '/autonav/obstacles', self.on_obstacles_received, 10)
        self.position_subscriber = self.create_subscription(Position, '/autonav/position', self.on_position_received, 10)
        self.msg = Path()
        self.robo_position = Position()
        self.robo_position_wp = [0,0,0,2]
        self.pixels_to_meters = (4/640, 4/480)
        self.robo_and_gps_path = [self.robo_position_wp]
        self.planned_path = []
        self.gps_waypoints = [[0, 10, 0, 2]]
        self.normalized_theta = 0.0
        self.min_distance = .5

    def reset(self):
        self.robo_and_gps_path = [self.robo_position_wp]
        self.planned_path = []

    def set_drive_point(self):
        raw_theta = self.robo_position.theta 
        if raw_theta >= 0:
            self.normalized_theta = raw_theta % (2 * math.pi)
        elif raw_theta < 0:
            self.normalized_theta = raw_theta % (-1 * 2 * math.pi)
        # these are flipped because our theta is shifted 90 degrees
        
        dpx, dpy = self.rotate(self.robo_position_wp[0], self.robo_position_wp[1] + 2, self.robo_position_wp[0], self.robo_position_wp[1])
        drive_point = [dpx, dpy, 1, 1]
        return drive_point

    def set_unplanned_path(self):
        self.robo_and_gps_path = []
        self.robo_and_gps_path.append(self.robo_position_wp)
        self.robo_and_gps_path.append(self.set_drive_point())
        for wp in self.gps_waypoints:
            self.robo_and_gps_path.append(wp)
        self.get_logger().info(f'THE LOCAL UNPLANNED PATH IS {self.robo_and_gps_path}')
        

    def rotate(self, x, y, centerx, centery):
        x1 = ((x - centerx) * math.cos(self.normalized_theta)) - ((y - centery) * math.sin(self.normalized_theta)) + centerx
        y1 = ((x - centerx) * math.sin(self.normalized_theta)) + ((y - centery) * math.cos(self.normalized_theta)) + centery
        
        return x1, y1
    
    def path_plan(self, local_obstacles, direction):
        test = tangent_based.path_planning()
        test.setpath(self.robo_and_gps_path)
        test.setobstacles(local_obstacles)
        
        self.get_logger().info("chuggin in path_planning")
        """counter = 0
        while(test.updated == True):
            if counter < 3:
                test.intersections(direction)
                test.path_intersections()
                test.delete_inside()
                counter = counter + 1
            else:
                break

        print("done chuggin")
        test.path_intersections()
        test.delete_inside()"""

        for i in range(1):
            #print("intersections called")
            test.intersections(direction)
            test.path_intersections()
            test.delete_inside()

        return test.final


    def set_path(self, local_path):
        self.msg.path_data = []
        for local_waypoints in local_path:
            waypoint = Waypoint()
            waypoint.x, waypoint.y, waypoint.is_generated, waypoint.can_be_deleted = float(local_waypoints[0]), float(local_waypoints[1]), int(local_waypoints[2]), int(local_waypoints[3])
            self.msg.path_data.append(waypoint)

    def on_position_received(self, position = Position):
        self.robo_position_wp = [position.x, position.y, 0, 2]
        self.robo_position = position 
        #self.get_logger().info(f"Setting the position to {position}")

    def publish_path(self):
        self.publisher.publish(self.msg)
        self.get_logger().info(f"publishing {self.msg} as Path to /autonav/Path")

    def on_obstacles_received(self, Obstacles):
        direction = "ccw" # change later based on pose
        local_obstacles = []
        obstacles_data = Obstacles.obstacles_data
        self.set_unplanned_path()
        for obstacle in obstacles_data:
            x = ((obstacle.center_x - (640/2)) * self.pixels_to_meters[0]) + self.robo_position_wp[0]
            y = ((obstacle.center_y - (480/2)) * self.pixels_to_meters[0]) + self.robo_position_wp[1] + self.min_distance
            x, y = self.rotate(x,y, self.robo_position_wp[0], self.robo_position_wp[1])
            rad = obstacle.radius * self.pixels_to_meters[1]
            #local_obstacles.append([((obstacle.center_x - (640/2)) * self.pixels_to_meters[0]) + self.robo_position_wp[0], ((obstacle.center_y - (480/2)) * self.pixels_to_meters[0]) + self.robo_position_wp[1], obstacle.radius * self.pixels_to_meters[1]])
            if rad >= .3:
                local_obstacles.append([x,y,rad])
        
        self.get_logger().info(f"I heard {local_obstacles} as the local obstacles")
        self.planned_path = self.path_plan(local_obstacles, direction)
        self.set_path(self.planned_path)
        self.publish_path()
        planning_test.planning_test(self.robo_and_gps_path, local_obstacles)


def isInside(circle_x, circle_y, rad, x, y):
    if ((x - circle_x) * (x - circle_x) +
        (y - circle_y) * (y - circle_y) <= rad * rad):
        return True
    else:
        return False
    
# returns a random set of waypoints, obstacles, and a radius around those obstacles to path plan around them.
def get_random_path_planning_simulation():
    rand_wps = []
    rand_path_length = random.randint(4, 6)
    for i in range(rand_path_length):
        rand_x = random.randint(-5, 5)
        rand_y = random.randint(-5, 5)
        rand_wps.append([rand_x, rand_y, 0, 1])

    

    rand_obstacles = []
    rand_amount_obstacles = random.randint(30, 40)
    for i in range(rand_amount_obstacles):
        rand_safety_d = random.uniform(.5, .75)
        rand_obst_x = random.uniform(-5, 5)
        rand_obst_y = random.uniform(-5, 5)
        rand_obstacles.append([rand_obst_x, rand_obst_y, rand_safety_d])

    # removing waypoints that are inside obstacles
    for_removal = []
    for wps in rand_wps:
        for obstacles in rand_obstacles:
            if isInside(obstacles[0], obstacles[1], obstacles[2], wps[0], wps[1]):
                if wps not in for_removal:
                    for_removal.append(wps)
    for waypoints in for_removal:
        if waypoints in rand_wps:
            rand_wps.remove(waypoints)

    rand_wps[0][2] = 2
    rand_wps[len(rand_wps) - 1][2] = 2
    
    print(f"removed {len(for_removal)} points")
    print(f"Length of rand_wps after removal {len(rand_wps)}")
    print(f"rand_wps is now {rand_wps}")
    #print(f"Length of rand_wps {len(rand_wps)} length of rand_obstacles {len(rand_obstacles)}")

    
    return rand_wps, rand_obstacles, rand_safety_d

def main(args=None):
    
    
    rclpy.init(args=args)
    path_planner = PathPlanner()
    generated_path = get_random_path_planning_simulation()
    """planning_test.planning_test(generated_path[0], generated_path[1])
    pathccw = tangent_based.path_planning()
    pathccw.setpath(generated_path[0])
    pathccw.setobstacles(generated_path[1])

    counter = 0
    while(pathccw.updated == True):
        if counter < 10:
            pathccw.intersections("ccw")
            pathccw.path_intersections()
            pathccw.delete_inside()
            counter = counter + 1
        else:
            break

    pathccw.path_intersections()
    pathccw.delete_inside()

    print(f"PATHCCW.FINAL {pathccw.final}")
    path_planner.set_path(pathccw.final)"""


    rclpy.spin(path_planner)
    path_planner.destroy_node
    rclpy.shutdown

    

if __name__ == "__main__":
    main()
    # this generates a random set of parameters for the path planner
    
    
    
    
    # commented out is a hardcoded test
    #waypoints = [(1,0), (2,2), (3,3)]
    #obstacles = [(1.5, 1.5), (5, 5)]
    #path_planning_test.planning_test(waypoints, obstacles, .5)


