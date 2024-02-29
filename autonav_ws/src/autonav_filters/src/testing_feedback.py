import numpy as np
import math
import random

if __name__ == "__main__":
    n = 16
    M = 20
    result = 16 % 10

    print(F"{result}")

    cosine_result = math.cos(2 * np.pi)
    
    print(F"{cosine_result}")

    #sum_theta_x = 0
    sum_theta_x = -7.771561172376096e-16


    #sum_theta_y = 0
    sum_theta_y = -6.286637876939949e-15

    sum_weight = 2
    
    
    avg_theta = math.atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight) % (2 * math.pi)
    print(f"{avg_theta}")