import numpy as np
import math
import random

# exponential

# longer number is 0.0633460851727573
dist_sqrt =.063345

exponential_result = math.exp(-dist_sqrt / (2 * 0.45 ** 2))
print(f"exponential result: {exponential_result}")

# normal distribution
theta_init = 0.0

theta = theta_init

for i in range(10):
    theta = np.random.normal(theta, 0.05) % (2 * math.pi)

    print(f"theta: {theta}\n")