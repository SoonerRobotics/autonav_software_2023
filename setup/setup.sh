#!/bin/bash

sudo apt update

# Common dependencies
bash common.sh

# Vectornav Dependencies
bash vnav.sh

# Rosdep
sudo apt-get install python3-rosdep2
rosdep install --from-paths ../autonav_ws/src --ignore-src -y --rosdistro=humble

# boot time
echo "Use 'systemctl enable autonav' to enable the service at boot time."