import pure_pursuit
import matplotlib.pyplot as plt
import numpy as np

# uses pyplot to test the efficacy of a pure pursuit algorithm
# takes robot position, a set of waypoints, and the lookahead distance as arguments
def pursuit_test(robo_pos, wpoints, r):
    test = pure_pursuit.pure_pursuit()
    test.setpath(wpoints)
    lookahead_point = test.get_lookahead_point(robo_pos[0], robo_pos[1], r)

    print(f'On path {wpoints} when the robot is at {robo_pos} the lookahead point is {lookahead_point}')

    look_circle = plt.Circle(robo_pos, r, fill=False)

    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.add_patch(look_circle)

    x, y = zip(*wpoints)

    ax.plot(x, y, '-o', label='Path', zorder=1)

    ax.scatter(wpoints[len(wpoints)-1][0], wpoints[len(wpoints)-1][1], color='Yellow', label='End point', zorder=2)
    print(f'wpoints end: {wpoints[len(wpoints)-1][0]} x {wpoints[len(wpoints)-1][1]} y ')

    ax.scatter(wpoints[0][0], wpoints[0][1], color='mediumorchid', label='Start point')

    ax.scatter(lookahead_point[0], lookahead_point[1], color='green', label='Optimal goal point', zorder=3)

    ax.scatter(robo_pos[0], robo_pos[1], color='red', label='Robot position')

    plt.title(f'Far intersection for robot @ {robo_pos} and look distance {r:.2f}')
    plt.xlabel('x')
    plt.ylabel('y', rotation=0)
    plt.legend()

    plt.show()
