import pure_pursuit
import matplotlib.pyplot as plt
import numpy as np
import random

def pursuit_test(robo_pos, wpoints, r):
    test = pure_pursuit.pure_pursuit()
    test.setpath(wpoints)
    lookahead_point = test.get_lookahead_point(robo_pos[0], robo_pos[1], 4)

    print(f'On path {wpoints} when the robot is at {robo_pos} the lookahead point is {lookahead_point}')

    look_circle = plt.Circle(robo_pos, r, fill=False)

    fig, ax = plt.subplots()
    ax.set_xlim(r * -3, r * 3)
    ax.set_ylim(r * -3, r * 3)
    ax.add_patch(look_circle)

    x, y = zip(*wpoints)

    ax.plot(x, y, '-o', label='Path')

    ax.scatter(robo_pos[0], robo_pos[1], color='red', label='Robot position')
    ax.scatter(lookahead_point[0], lookahead_point[1], color='green', label='Optimal goal point')

    plt.title(f'Far intersection for robot @ {robo_pos} and look distance {r}')
    plt.xlabel('x')
    plt.ylabel('y', rotation=0)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    rand_wps = []
    rand_path_length = random.randint(1,20)
    for i in range(rand_path_length):
        rand_x = random.randint(-10, 10)
        rand_y = random.randint(-10, 10)
        rand_wps.append((rand_x, rand_y))
    rand_robo_pos = random.randint(-5, 5)
    rand_r = random.randint(1, 4)


    waypoints = [(0, 0), (2, 3), (4, 6), (4, 5), (3,0), (6,1)]
    #pursuit_test((1,1), waypoints, 4)
    pursuit_test(rand_robo_pos, rand_wps, rand_r)
    