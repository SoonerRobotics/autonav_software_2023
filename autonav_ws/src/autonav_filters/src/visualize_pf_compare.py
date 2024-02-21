import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

python_data = np.genfromtxt("py_gps_log.txt", delimiter=", ")
cpp_data = np.genfromtxt("gps_log_file.txt", delimiter=", ")

#py_xs = [python_data[0][i] for i in range(len(python_data))]
print(python_data)
print(cpp_data)

print(python_data.shape)
print(python_data.shape[1])
print(cpp_data.shape)

# python paths
fig, (ax1, ax2) = plt.subplots(1,2)
ax1.set_xlabel("python")
for i in range(int(python_data.shape[1] / 2)):
    print(i)
    print(i * 2 + 1)
    ax1.plot(python_data[:, i*2].tolist(), python_data[:, (i*2)+1].tolist(), label=f'python path {i}')


# cpp data
    ax2.set_xlabel("cpp")
for i in range(int(python_data.shape[1] / 2)):
    print(i)
    print(i * 2 + 1)
    ax2.plot(cpp_data[:, i*2].tolist(), cpp_data[:, (i*2)+1].tolist(), label=f'cpp path {i}')

plt.plot(cpp_data[:, 1].tolist(), cpp_data[:, 0].tolist(), '--', label="cpp")

ax1.legend()
ax2.legend()

plt.show()