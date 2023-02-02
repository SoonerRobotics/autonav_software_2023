import numpy as np
import math

class path_planning:
    path = []
    obstacles = []
    final = []

    def initialize(self, wpoints, obsts):
        self.path = wpoints.copy()
        self.obstacles = obsts.copy()
        self.final = wpoints.copy()
    
    def setpath(self, list):
        self.path = list.copy()
        self.final = list.copy()

    def setobstacles(self, list):
        self.obstacles = list
    
    def add_to_path(self, x, y):
        self.path.append((x, y))
    
    def add_to_obstacles(self, x, y):
        self.path.append((x, y))

    def clamp(self, arg):
        if arg > 1:
            arg = 1
        elif arg < -1:
            arg = -1

        return arg

    # for an understanding of why these changes to the result of the tangent function are necessary, see: https://www.mathsisfun.com/polar-cartesian-coordinates.html
    def quadrant(self, theta, x, y):
        print("quadrant getting called")
        print(x)
        print(y)
        if (x < 0.0 and y > 0.0):
            print("Second quadrant, adding pi")
            theta += math.pi
        elif (x < 0.0 and y < 0.0):
            theta += math.pi
            print("Third quadrant, adding pi")
        elif (x > 0.0 and y < 0.0):
            theta += (math.pi * 2)
            print("Fourth quadrant, adding 2pi")

        return theta

    # check for double intersections of the path of waypoints with the obstacles
    def intersections(self, safety_d):
        original_path_length = len(self.path.copy())

        for i in range(len(self.path)-1):
            # draw the segment
            seg_start = self.path[i]
            seg_end = self.path[i+1]

            # intersect the obstacles with the segment
            for j in range(len(self.obstacles.copy())):
                intersecting_point_1 = None
                intersecting_point_2 = None

                # translate the segment to the origin
                p1 = seg_start[0] - self.obstacles[j][0], seg_start[1] - self.obstacles[j][1]
                p2 = seg_end[0] - self.obstacles[j][0], seg_end[1] - self.obstacles[j][1]

                # get dr, D, sgn(dy) and the discriminant to compute intersections from https://mathworld.wolfram.com/Circle-LineIntersection.html
                # first implemented by https://github.com/xiaoxiae/PurePursuitAlgorithm/blob/master/src/main/PurePursuit.java
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]

                dr = math.sqrt((dx**2 + dy**2))
                D = (p1[0]*p2[1]) - (p2[0]*p1[1])

                discrim = safety_d**2 * dr**2 - D**2

                if dy < 0:
                    sgn_dy = -1
                else:
                    sgn_dy = 1

                # if the discriminant is < 0, the segment does not intersect the circle. If it = 0 then it is tangent to the circle
                if discrim <= 0 or p1 == p2:
                    continue

                # x and y coordinates of the intersections are solved with the quadratic formula, giving 2 points, labeled _add and _sub respectively
                # x1 = (D * dy + sign(dy) * dx * math.sqrt(discriminant)) / (d * d)
                new_x_add = ((D * dy) + (sgn_dy * dx * math.sqrt(discrim))) / dr**2
                new_y_add = ((-D * dx) + (abs(dy) * math.sqrt(discrim))) / dr**2

                new_x_sub = ((D * dy) - (sgn_dy * dx * math.sqrt(discrim))) / dr**2
                new_y_sub = ((-D * dx) - (abs(dy) * math.sqrt(discrim))) / dr**2
                print(f"new_x_add is {new_x_add}\nnew_y_add is {new_y_add}\nnew_x_sub is {new_x_sub}\nnew_y_sub is {new_y_sub}")
                print(f"obstacles[j][x] {self.obstacles[j][0]}")
                print(f"obstacles[j][y] {self.obstacles[j][1]}")

                # check that the new point is actually on the segment
                # segment min x < goal x < segment max x, or segment min y < goal y < segment max y
                print(f'segment_min_x {min(p1[0], p2[0])} < new_x_add {new_x_add} < segment_max_x {max(p1[0], p2[0])}')
                valid_intersection_add = min(p1[0], p2[0]) < new_x_add and new_x_add < max(p1[0], p2[0]) or min(p1[1], p2[1]) < new_y_add and new_y_add < max(p1[1], p2[1])
                valid_intersection_sub = min(p1[0], p2[0]) < new_x_sub and new_x_sub < max(p1[0], p2[0]) or min(p1[1], p2[1]) < new_y_sub and new_y_sub < max(p1[1], p2[1])

                # check that the new point is actually on the segment
                # segment min x < goal x < segment max x, or segment min y < goal y < segment max y
                #print(f'segment_min_x {min(p1[0], p2[0])} < new_x_add {new_x_add} < segment_max_x {max(p1[0], p2[0])}')
                #valid_intersection_add = min(p1[0], p2[0]) < new_x_add and new_x_add < max(p1[0], p2[0]) or min(p1[1], p2[1]) < new_y_add and new_y_add < max(p1[1], p2[1])
                #valid_intersection_sub = min(p1[0], p2[0]) < new_x_sub and new_x_sub < max(p1[0], p2[0]) or min(p1[1], p2[1]) < new_y_sub and new_y_sub < max(p1[1], p2[1])

                # intersections are only tangent to the segment
                if valid_intersection_add or valid_intersection_sub:
                    intersecting_point = None
                
                # second check for tangency, after limiting the intersections to those only on the line segment
                if valid_intersection_add and valid_intersection_sub != None:
                    intersecting_point_1 = (new_x_add, new_y_add)
                    intersecting_point_2 = (new_x_sub, new_y_sub)
                    swapper = (0, 0)
                    # check cartesian distance from p1 to the first intersection candidate
                    distance_to_1 = math.sqrt((p1[0] - intersecting_point_1[0])**2 + (p1[1] - intersecting_point_1[1])**2)
                    distance_to_2 = math.sqrt((p1[0] - intersecting_point_2[0])**2 + (p1[1] - intersecting_point_2[1])**2)
                    print(f"distance_to_1 {distance_to_1}\ndistance_to_2 {distance_to_2}")
                    print(f"intersecting_point_1x {intersecting_point_1[0] + self.obstacles[j][0]} intersecting_point_1y {intersecting_point_1[1] + self.obstacles[j][1]}")
                    print(f"intersecting_point_2x {intersecting_point_2[0] + self.obstacles[j][0]} intersecting_point_2y {intersecting_point_2[1] + self.obstacles[j][1]}")
                    # if intersecting_point_2 is closer, it is actually the first intersection
                    if distance_to_1 > distance_to_2:
                        print("swapping")
                        swapper = intersecting_point_1
                        intersecting_point_1 = intersecting_point_2
                        intersecting_point_2 = swapper

                # populate the circle with waypoints around the safety_d of the object
                points = []
                # theta value of first intersection
                # draw points around the circle
                if intersecting_point_1 and intersecting_point_2 != None:

                    # clamp acos and asin arguments to -1 or 1 so that floating point error doesn't ruin the program
                    theta1t_arg = math.atan((intersecting_point_1[1]) / (intersecting_point_1[0]))
                    theta2t_arg = math.atan((intersecting_point_2[1]) / (intersecting_point_2[0]))

                    # check quadrant
                    print(f"theta1t_arg {theta1t_arg}")
                    print(f"theta2t_arg {theta2t_arg}")
                    # check quadrant

                    print(f"initially {theta1t_arg} {theta2t_arg}")
                    theta1t_arg = self.quadrant(theta1t_arg, intersecting_point_1[0], intersecting_point_1[1])
                    theta2t_arg = self.quadrant(theta2t_arg, intersecting_point_2[0], intersecting_point_2[1])
                    print(f"after quadrant checking {theta1t_arg} {theta2t_arg}")


                    print(f"safety_d {safety_d}")
                    print(f"Fake circle intersecting point 1 polar {(safety_d * math.cos(theta1t_arg))}, {(safety_d * math.sin(theta1t_arg))}")
                    print(f"Fake circle intersecting point 1 cartesian {(intersecting_point_1[0])}, {(intersecting_point_1[1])}")
                    print(f"Fake circle intersecting point 2 polar {(safety_d * math.cos(theta2t_arg))}, {(safety_d * math.sin(theta2t_arg))}")
                    print(f"Fake circle intersecting point 2 cartesian {(intersecting_point_2[0])}, {(intersecting_point_2[1])}")
                    print(f"1st intersection point polar {(safety_d * math.cos(theta1t_arg)) + self.obstacles[j][0]}, {(safety_d * math.sin(theta1t_arg) + self.obstacles[j][1])}")
                    print(f"1st intersection point cartesian {(intersecting_point_1[0]) + self.obstacles[j][0]}, {(intersecting_point_1[1]) + self.obstacles[j][1]}")
                    print(f"2nd intersection point polar {(safety_d * math.cos(theta2t_arg)) + self.obstacles[j][0]}, {(safety_d * math.sin(theta2t_arg) + self.obstacles[j][1])}")
                    print(f"2nd intersection point cartesian {(intersecting_point_2[0]) + self.obstacles[j][0]}, {(intersecting_point_2[1]) + self.obstacles[j][1]}")
                    print(f"theta1t_arg {theta1t_arg}")
                    print(f"theta2t_arg {theta2t_arg}")

                    # get two different dtheta5ts, one for traversing the circle clockwise and the other for traversing it counter-clockwise. Pick whichever is smaller and use that.
                    # dtheta5t is the same, but the arclengths for left and right will be different
                    # really I just want to travel closer to the center of the path, if possible
                    dtheta_counter_clockwise = theta2t_arg - theta1t_arg
                    print(f"dtheta_counter_clockwise originally {dtheta_counter_clockwise}")

                    # normalize the counterclockwise angle difference so everything makes sense
                    if dtheta_counter_clockwise < 0:
                        dtheta_counter_clockwise += 2 * math.pi
                    
                    print(f"dtheta_counter_clockwise after checking for negative {dtheta_counter_clockwise}")

                    # subtracting from the argument of cos and sin incrementally gives clockwise motion. 
                    # The clockwise angle difference is just the rest of the circle excluded by the counterclockwise difference
                    dtheta_clockwise = -1 * ((2 * math.pi) - dtheta_counter_clockwise)
                    print(f"dtheta_clockwise {dtheta_clockwise}")

                    # go clockwise or counter-clockwise depending on which theta is smaller
                    if abs(dtheta_counter_clockwise) < abs(dtheta_clockwise):
                        print("Counter-clockwise is shorter")
                        dtheta5t = dtheta_counter_clockwise / 5
                    else:
                        print("Clockwise is shorter")
                        dtheta5t = dtheta_clockwise / 5
                    
                    print(f"dtheta5t {dtheta5t}")

                    # this for loop draws points along the edge of the avoidance circles
                    for k in range(0, 5):
                        print(f"dtheta5t * k {dtheta5t * (k)}")
                        points.append((((((safety_d+ .1) * (math.cos(theta1t_arg + (dtheta5t * (k))))) + self.obstacles[j][0]), (((safety_d + .1)* math.sin(theta1t_arg + (dtheta5t * (k)))) + self.obstacles[j][1]))))
                        print(f"point is {points}")
                        print(len(self.final))
                    
                    # inserts the points in the right order into the waypoint list
                    for l in range(len(points)):
                        self.final.insert(i + len(self.final) - original_path_length + 1, points[l])
        
        print(f"obstacles {self.obstacles}")        
        print(f"original path {self.path}")
        print(f"final path {self.final}")