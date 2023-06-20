import numpy as np
from typies import Feedback, GPS
import matplotlib.pyplot as plt
import math
import os

HOME_DIR = os.path.expanduser("~")
FOLDER = os.path.join(HOME_DIR, "day1")

runs = {}
# For each folder, create a new run
for folder in os.listdir(FOLDER):
    if folder.startswith("run"):
        run = folder
        runs[run] = []

        # Read the ENTRY_GPS.csv file
        gps_file = os.path.join(FOLDER, folder, "ENTRY_POSITION.csv")
        with open(gps_file, "r") as f:
            lines = f.readlines()
            for line in lines:
                lat = float(line.split(",")[5])
                lon = float(line.split(",")[6])
                runs[run].append((lat, lon))

for run in runs:
    lats = [gps[0] for gps in runs[run]]
    lons = [gps[1] for gps in runs[run]]
    plt.plot(lons, lats, label=run)

waypoints = [(42.6682697222 ,-83.2193403028),(42.6681206444,-83.2193606083),(42.6680766333,-83.2193591583),(42.6679277056,-83.2193276417), (42.6681268, -83.218887)]
# Plot as dots
lats = [gps[0] for gps in waypoints]
lons = [gps[1] for gps in waypoints]
plt.plot(lons, lats, "o", label="Waypoints")

plt.legend()
plt.show()