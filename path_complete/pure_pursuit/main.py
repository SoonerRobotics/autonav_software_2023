import pure_pursuit
import pursuit_test
import rclpy
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


if __name__ == "__main__":
    # commented out is a hard coded test
    waypoints = [(-4, -2), (-4, 2), (-2, 2), (-2, -2), (0, -2), (0, 2), (2, 2), (2, -2)]
    pursuit_test.pursuit_test((-0.5, 0), waypoints, 2.3)
    
    # test using random simulated position, path, and radius
    #test1 = get_random_pursuit_simulation()
    #pursuit_test.pursuit_test(test1[0], test1[1], test1[2])