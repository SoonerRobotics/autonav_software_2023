import matplotlib.pyplot as plt
import numpy as np
import math
import path_planning

def planning_test(wpoints, obstacles, safety_d):
    test = path_planning.path_planning()
    test.setpath(wpoints)
    test.setobstacles(obstacles)
    test.intersections(safety_d)

    obstacle_circles = []
    for i in range(len(obstacles)):
        obstacle_circles.append(plt.Circle(obstacles[i], safety_d, fill=False, zorder=2))

    
    fig, ax = plt.subplots()

    ax.set_ylim(-10, 10)
    ax.set_xlim(-10, 10)

    for i in range(len(obstacle_circles)):
        ax.add_patch(obstacle_circles[i])

    x, y = zip(*test.final)

    ax.scatter(x[0], y[0], label = 'Start', color= 'Green', zorder=2)
    ax.scatter(x[len(x) - 1], y[len(y) - 1], label = 'End', color='Red', zorder = 3)

    ax.plot(x, y, '-o', label='Path', zorder=1)

    
    plt.legend()
    plt.show()