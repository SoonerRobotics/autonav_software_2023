import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

python_data = np.genfromtxt("py_gps_log.txt", delimiter=", ")
cpp_data = np.genfromtxt("gps_log_file.txt", delimiter=", ")

#py_xs = [python_data[0][i] for i in range(len(python_data))]
print(python_data)
print(cpp_data)

print(python_data.shape)
print(cpp_data.shape)

plt.plot(python_data[:, 1].tolist(), python_data[:, 0].tolist(), label='python')
plt.plot(cpp_data[:, 1].tolist(), cpp_data[:, 0].tolist(), '--', label="cpp")

plt.legend()

plt.show()