#!/bin/bash

sudo apt update
sudo apt install wget unzip -y

# Common dependencies
bash common.sh

# Vectornav Dependencies
bash vnav.sh

# Steam Controller Dependencies
bash steam.sh

# Submodules
git submodule update --init --recursive
touch imgui/COLCON_IGNORE

# ImGUI Dependencies
sudo apt install libglfw3-dev libglew-dev -y

# Rosdep
# sudo apt-get install python3-rosdep2 -y
# rosdep install --from-paths ../autonav_ws/src --ignore-src -y --rosdistro=humble

# boot time
echo "Use 'systemctl enable autonav' to enable the service at boot time."