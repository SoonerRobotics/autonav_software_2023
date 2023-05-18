import TangentBased
import PlanningTest
import random
import time


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
        rand_wps.append([rand_x, rand_y, 0, 0])

    

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


if __name__ == "__main__":
    
    # this generates a random set of parameters for the path planner
    path_test = get_random_path_planning_simulation()
    
    PlanningTest.planning_test(path_test[0], path_test[1])
    
    
    
    # commented out is a hardcoded test
    #waypoints = [(1,0), (2,2), (3,3)]
    #obstacles = [(1.5, 1.5), (5, 5)]
    #path_planning_test.planning_test(waypoints, obstacles, .5)

