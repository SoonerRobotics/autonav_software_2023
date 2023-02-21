import path_planning
import path_planning_test
import random

# returns a random set of waypoints, obstacles, and a radius around those obstacles to path plan around them.
def get_random_path_planning_simulation():
    rand_wps = []
    rand_path_length = random.randint(4, 6)
    for i in range(rand_path_length):
        rand_x = random.randint(-5, 5)
        rand_y = random.randint(-5, 5)
        rand_wps.append((rand_x, rand_y, 0))

    rand_obstacles = []
    rand_amount_obstacles = random.randint(4, 4)
    for i in range(rand_amount_obstacles):
        rand_safety_d = random.uniform(.5, .75)
        rand_obst_x = random.uniform(-5, 5)
        rand_obst_y = random.uniform(-5, 5)
        rand_obstacles.append([rand_obst_x, rand_obst_y, rand_safety_d])
    


    # deleting waypoints that are inside of objects
    print("This is where removal statements would be")
    
    print(f"Length of rand_obstacles before removal {len(rand_obstacles)}")
    print(f"rand_obstacles is {rand_obstacles}")
    for_removal = []
    for i in range(len(rand_obstacles)):
        for j in range(len(rand_obstacles)):
            if (rand_obstacles[i][0] >= ((rand_obstacles[j][0] - rand_safety_d)/2) and rand_obstacles[i][0] <= ((rand_obstacles[j][0] + rand_safety_d)*2)) and (rand_obstacles[i][1] >= ((rand_obstacles[j][1] - rand_safety_d)/2) and rand_obstacles[i][1] <= ((rand_obstacles[j][1] + rand_safety_d)*2)):
                if i not in for_removal and i != j:
                    for_removal.append(rand_obstacles[i])
    for i in for_removal:
        if i in rand_obstacles:
            rand_obstacles.remove(i)

    print(f"removed {len(for_removal)} points")
    print(f"Length of rand_obstacles after removal {len(rand_obstacles)}")
    print(f"rand_obstacles is now {rand_obstacles}")

    print(f"Length of rand _wps before removal {len(rand_wps)}")
    print(f"rand_wps is {rand_wps}")

    for_removal = []
    for i in range(len(rand_wps)):
        for j in range(len(rand_obstacles)):
            if (rand_wps[i][0] >= ((rand_obstacles[j][0] - rand_safety_d)/2) and rand_wps[i][0] <= ((rand_obstacles[j][0] + rand_safety_d)*2)) and (rand_wps[i][1] >= ((rand_obstacles[j][1] - rand_safety_d)/2) and rand_wps[i][1] <= ((rand_obstacles[j][1] + rand_safety_d)*2)):
                if i not in for_removal:
                    for_removal.append(rand_wps[i])
    for i in for_removal:
        if i in rand_wps:
            rand_wps.remove(i)

    
        
    print(f"removed {len(for_removal)} points")
    print(f"Length of rand_wps after removal {len(rand_wps)}")
    print(f"rand_wps is now {rand_wps}")
    #print(f"Length of rand_wps {len(rand_wps)} length of rand_obstacles {len(rand_obstacles)}")

    return rand_wps, rand_obstacles, rand_safety_d


if __name__ == "__main__":
    
    # this generates a random set of parameters for the path planner
    path_test = get_random_path_planning_simulation()
    path_planning_test.planning_test(path_test[0], path_test[1])
    
    # commented out is a hardcoded test
    #waypoints = [(1,0), (2,2), (3,3)]
    #obstacles = [(1.5, 1.5), (5, 5)]
    #path_planning_test.planning_test(waypoints, obstacles, .5)
