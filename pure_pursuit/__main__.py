import pure_pursuit
import pursuit_test
import path_planning_test
import matplotlib.pyplot as plt
import numpy as np
import random

# returns a random robot position, random set of waypoints, and random radius
def get_random_pursuit_simulation():
    rand_wps = []
    rand_path_length = random.randint(4, 10)
    for i in range(rand_path_length):
        rand_x = random.randint(-5, 5)
        rand_y = random.randint(-5, 5)
        rand_wps.append((rand_x, rand_y))
    rand_robo_pos = random.randint(-2,2), random.randint(-2,2)
    rand_r = random.uniform(.5, 3)

    return rand_robo_pos, rand_wps, rand_r


# returns a random set of waypoints, obstacles, and a radius around those obstacles to path plan around them.
def get_random_path_planning_simulation():
    rand_wps = []
    rand_path_length = random.randint(4, 10)
    for i in range(rand_path_length):
        rand_x = random.randint(-5, 5)
        rand_y = random.randint(-5, 5)
        rand_wps.append((rand_x, rand_y))

    rand_obstacles = []
    rand_amount_obstacles = random.randint(4, 10)
    for i in range(rand_amount_obstacles):
        rand_obst_x = random.uniform(-5, 5)
        rand_obst_y = random.uniform(-5, 5)
        rand_obstacles.append((rand_obst_x, rand_obst_y))
    rand_safety_d = random.uniform(.5, .75)


    # deleting waypoints that are inside of objects
    print("This is where removal statements would be")
    print(f"Length of rand _wps before removal {len(rand_wps)}")
    print(f"rand_wps is {rand_wps}")
    for_removal = []
    for i in range(len(rand_wps)):
        for j in range(len(rand_obstacles)):
            if (rand_wps[i][0] >= (rand_obstacles[j][0] - rand_safety_d) and rand_wps[i][0] <= (rand_obstacles[j][0] + rand_safety_d)) and (rand_wps[i][1] >= (rand_obstacles[j][1] - rand_safety_d) and rand_wps[i][1] <= (rand_obstacles[j][1] + rand_safety_d)):
                if i not in for_removal:
                    for_removal.append(i)
    for i in for_removal:
        del rand_wps[i]
        
    print(f"removed {len(for_removal)} points")
    print(f"Length of rand_wps after removal {len(rand_wps)}")
    print(f"rand_wps is now {rand_wps}")
    #print(f"Length of rand_wps {len(rand_wps)} length of rand_obstacles {len(rand_obstacles)}")

    return rand_wps, rand_obstacles, rand_safety_d


if __name__ == "__main__":
    # commented out is a hard coded test
    #waypoints = [(0, 0), (2, 3), (4, 6), (4, 5), (3,0), (6,1)]
    #pursuit_test.pursuit_test((1,1), waypoints, 4)
    
    # test using random simulated position, path, and radius
    test1 = get_random_pursuit_simulation()
    pursuit_test.pursuit_test(test1[0], test1[1], test1[2])
    
    path_test = get_random_path_planning_simulation()
    path_planning_test.planning_test(path_test[0], path_test[1], path_test[2])
    #waypoints = [(1,0), (2,2), (3,3)]
    #obstacles = [(1.5, 1.5), (5, 5)]
    #path_planning_test.planning_test(waypoints, obstacles, .5)
    