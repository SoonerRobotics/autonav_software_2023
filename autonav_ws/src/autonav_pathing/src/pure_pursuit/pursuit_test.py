#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import lookahead_finder

# uses pyplot to test the efficacy of a pure pursuit algorithm
# takes robot position, a set of waypoints, and the lookahead distance as arguments
def pursuit_test(robo_pos, wpoints, r):
    test = lookahead_finder.PurePursuit()

    # set the waypoints on the path
    test.setpath(wpoints)

    # get the coordinates of the lookahead point on the path that intersects the robots look distance, r
    lookahead_point = test.get_lookahead_point(robo_pos[0], robo_pos[1], r)

    # describe the path, robot position, and lookahead point in the console
    #print(f'On path {wpoints} when the robot is at {robo_pos} the lookahead point is {lookahead_point}')

    # create a circle around the robot's position showing its lookahead distance
    look_circle = plt.Circle(robo_pos, r, fill=False)

    # create a subplot for plotting the test results
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    
    # plot the circle
    ax.add_patch(look_circle)

    # seperate the x and y values of the waypoints tuples into lists
    x, y = zip(*wpoints)

    # plot the waypoints of the path with a path line going from point to point
    ax.plot(x, y, '-o', label='Path', zorder=1)

    # plot a yellow point on the last waypoint of the path to help the user determine path direction
    ax.scatter(wpoints[len(wpoints)-1][0], wpoints[len(wpoints)-1][1], color='Yellow', label='End point', zorder=2)
    #print(f'wpoints end: {wpoints[len(wpoints)-1][0]} x {wpoints[len(wpoints)-1][1]} y ')

    # plot a purple point on the first waypoint of the path to help the user determine path direction
    ax.scatter(wpoints[0][0], wpoints[0][1], color='mediumorchid', label='Start point')

    # plot the lookahead point
    ax.scatter(lookahead_point[0], lookahead_point[1], color='green', label='Optimal goal point', zorder=3)

    # plot the robot's position
    ax.scatter(robo_pos[0], robo_pos[1], color='red', label='Robot position')

    # title the figure and axes
    plt.title(f'Far intersection for robot @ {robo_pos} and look distance {r:.2f}')
    plt.xlabel('x')
    plt.ylabel('y', rotation=0)
    
    # add a legend so that colored objects on the plot are understood
    plt.legend()

    # show the plot
    plt.show()
