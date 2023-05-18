import numpy as np
from filter import PFilter
from typies import Feedback, GPS
import matplotlib.pyplot as plt

FEEDBACK_FILE = "feedback.csv"
GPS_FILE = "gps.csv"

feedbacks = []
with open(FEEDBACK_FILE, "r") as f:
	for line in f.readlines():
		line = line.strip()
		if line == "" or line.split(",")[0] == "timestamp":
			continue
		feedbacks.append(line.split(","))
  
gps = []
with open(GPS_FILE, "r") as f:
	for line in f.readlines():
		line = line.strip()
		if line == "" or line.split(",")[0] == "timestamp":
			continue
		gps.append(line.split(","))
  
print(f"Loaded {len(feedbacks)} feedbacks and {len(gps)} gps points")

data = np.array(feedbacks + gps)
data = np.hstack((data, np.array([["feedback"]*len(feedbacks) + ["gps"]*len(gps)]).T))
data = data[data[:,0].argsort()]
plt.figure(figsize=(15,15))

newpf_poses = []
oldpf_poses = []
rawgps_poses = []

pf = PFilter()
first_gps = None
for row in data:
	if row[4] == "feedback":
		feedback = Feedback(float(row[1]), float(row[2]), float(row[3]))
		if float(row[1]) == 0 and float(row[2]) == 0 and float(row[3]) == 0:
			continue
		average = pf.feedback(feedback)
		if first_gps is not None:
			gps_x = first_gps.latitude + average[0] / 111086.2
			gps_y = first_gps.longitude - average[1] / 81978.2
			newpf_poses.append([gps_x, gps_y])

	else:
		gps = GPS(float(row[1]), float(row[2]))
		if first_gps is None:
			first_gps = gps
   
		pf.gps(gps)
		rawgps_poses.append([gps.latitude, gps.longitude])

# Filter out any [0, 0] points
newpf_poses = [x for x in newpf_poses if x[0] != 0 and x[1] != 0]
rawgps_poses = [x for x in rawgps_poses if x[0] != 0 and x[1] != 0]

plt.plot([x[0] for x in newpf_poses], [x[1] for x in newpf_poses], label="Particle Filter")
plt.plot([x[0] for x in rawgps_poses], [x[1] for x in rawgps_poses], label="Raw GPS")
plt.legend()
plt.show()