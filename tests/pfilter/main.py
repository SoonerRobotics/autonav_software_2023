import numpy as np
from filter import PFilter
from typies import Feedback, GPS
import matplotlib.pyplot as plt
import math

MAIN_FILE = "log.csv"
LAT_METER_CONV = 110944.21
LON_METER_CONV = 81978.2

data = []
with open(MAIN_FILE, "r") as f:
	for line in f.readlines():
		line = line.strip()
		bits = line.split(",")
		for i in range(len(bits)):
			bits[i] = bits[i].strip()
		if line == "" or bits[0] == "timestamp":
			continue
		
		type = bits[1]
		if type == "ENTRY_GPS" or type == "ENTRY_FEEDBACK":
			data.append(bits)

plt.figure(figsize=(15,15))

newpf_poses = []
rawgps_poses = []

print("Starting")

pf = PFilter()
first_gps = None
for row in data:
	if row[1] == "ENTRY_FEEDBACK":
		feedback = Feedback(float(row[2]), float(row[3]), float(row[4]))
		if float(row[2]) == 0 and float(row[3]) == 0 and float(row[4]) == 0:
			continue
		average = pf.feedback(feedback)
		if first_gps is not None:
			mtr_x = first_gps.latitude + average[0] / LAT_METER_CONV
			mtr_y = first_gps.longitude - average[1] / LON_METER_CONV
			newpf_poses.append([mtr_x, mtr_y, average[2]])

	else:
		gps = GPS(float(row[2]), float(row[3]))
		if gps.latitude == 0 or gps.longitude == 0:
			continue
		
		if first_gps is None:
			first_gps = gps
   
		pf.gps(gps)
		rawgps_poses.append([gps.latitude, gps.longitude])

print("ending")

# Filter out any [0, 0] points
newpf_poses = [x for x in newpf_poses if x[0] != 0 and x[1] != 0]
rawgps_poses = [x for x in rawgps_poses if x[0] != 0 and x[1] != 0]

print(str(len(newpf_poses)))

# Draw the pf path with quivers
for i in range(len(newpf_poses)):
	pose = newpf_poses[i]
	theta = pose[2]
	if i % 10 == 0:
		plt.quiver(pose[0], pose[1], math.cos(theta), math.sin(theta), color="red", scale=10)

plt.plot([x[0] for x in rawgps_poses], [x[1] for x in rawgps_poses], label="Raw GPS")
plt.plot([x[0] for x in newpf_poses], [x[1] for x in newpf_poses], label="PF")
plt.legend()
plt.show()
