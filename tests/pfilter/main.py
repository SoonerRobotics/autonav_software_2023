import numpy as np
from filter import PFilter
from deadrekt import DeadReckoning

import time
import matplotlib.pyplot as plt

FEEDBACK_FILE = "feedback.csv"
GPS_FILE = "gps.csv"

# Import feedback.csv and gps.csv
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

# Both gps and feedback have a timestamp column (the first one), so we can use that to align them. Create a mega array of all the data, add an extra column to indicate which data is from which file, and sort by timestamp.
data = np.array(feedbacks + gps)
data = np.hstack((data, np.array([["feedback"]*len(feedbacks) + ["gps"]*len(gps)]).T))
data = data[data[:,0].argsort()]

plt.figure(figsize=(10,10))

pf_poses = []
dr_poses = []

pf = PFilter()
dr = DeadReckoning()
for row in data:
	if row[4] == "feedback":
		if float(row[1]) == 0 and float(row[2]) == 0 and float(row[3]) == 0:
			continue
		# pf_pose = pf.feedback(row)
		dr_pose = dr.feedback(row)
		dr_poses.append(dr_pose)
	else:
		pf.gps(row)
  
plt.plot([x[0] for x in pf_poses], [x[1] for x in pf_poses], label="Particle Filter")
plt.plot([x[0] for x in dr_poses], [x[1] for x in dr_poses], label="Dead Reckoning")
plt.legend()
plt.show()