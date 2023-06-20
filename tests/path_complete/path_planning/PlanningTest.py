import matplotlib.pyplot as plt
import numpy as np
import math
import TangentBased
import time

def planning_test(wpoints, obstacles):
    testcw = TangentBased.path_planning()
    testcw.setpath(wpoints)
    testcw.setobstacles(obstacles)
    
    testccw = TangentBased.path_planning()
    testccw.setpath(wpoints)
    testccw.setobstacles(obstacles)

    # run test.intersections until no new points are added
    
    start_time = time.time()
    
    for i in range(10):
        #print("intersections called")
        testcw.intersections("cw")
        testcw.path_intersections()
        testcw.delete_inside()

        testccw.intersections("ccw")
        testccw.path_intersections()
        testccw.delete_inside()
    
    execution_time = time.time() - start_time
    
    

    # here
    print(f"Start test, test.final {testcw.final}")
    obstacle_circles = []
    obstacle_circles_2 = []
    for i in range(len(obstacles)):
        if i == 0:
            obstacle_circles.append(plt.Circle(obstacles[i], obstacles[i][2], fill=False, zorder=2, label="Obstacles"))
            obstacle_circles_2.append(plt.Circle(obstacles[i], obstacles[i][2], fill=False, zorder=2))
        else:
            obstacle_circles.append(plt.Circle(obstacles[i], obstacles[i][2], fill=False, zorder=2))
            obstacle_circles_2.append(plt.Circle(obstacles[i], obstacles[i][2], fill=False, zorder=2))
    fig, (ax1, ax2) = plt.subplots(1, 2)

    ax1.set_ylim(-10, 10)
    ax1.set_xlim(-10, 10)

    for i in range(len(obstacle_circles)):
        ax1.add_artist(obstacle_circles_2[i])
        ax2.add_patch(obstacle_circles[i])

    original_path = []
    for i in range(len(testcw.path)):
        original_path.append(testcw.path[i][0:2])

    ex, why = zip(*original_path)
    print(f"original_path {original_path}")

    ax1.set_ylim(-6, 6)
    ax1.set_xlim(-6, 6)

    ax1.scatter(ex[0], why[0], label = 'Path start', color= 'Green', zorder=2)
    ax1.scatter(ex[len(ex) - 1], why[len(why) - 1], label = 'Path end', color='Red', zorder = 3)

    ax1.plot(ex, why, '-o', label = 'Original path', zorder=1)

    ax1.set(xlabel='Before path planning')

    
    #print(f"test.final in PATH PLANNING TESTER {test.final}")

    final_path = []


    for i in range(len(testcw.final)):
        #print(f"test.final[i] {test.final[i]}")
        final_path.append(testcw.final[i][0:2])
    #print(f"final path in PATH PLANNING TESTER {final_path}")
    x, y = zip(*final_path)
    
    ax2.set_ylim(-6, 6)
    ax2.set_xlim(-6, 6)

    ax2.scatter(x[0], y[0], label = 'Path start', color= 'Green', zorder=2)
    ax2.scatter(x[len(x) - 1], y[len(y) - 1], label = 'Path end', color='Red', zorder = 3)

    ax2.plot(x, y, '-o', label='Waypoint path', zorder=1)

    ax2.set(xlabel='After path planning')

    print(f"Entire process time: {execution_time}")
    plt.legend()
    plt.show()

