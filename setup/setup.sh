#!/bin/bash

sudo apt update
sudo apt install wget unzip -y

# Common dependencies
bash common.sh

# Vectornav Dependencies
bash vnav.sh

# Steam Controller Dependencies
bash steam.sh

# ImGUI Dependencies
sudo apt install libglfw3-dev libglew-dev -y

# Python deps
sudo apt install python3-pip -y
pip3 install python-can[serial]

# Copy the udev rules to the correct location
sudo cp autonav.rules /etc/udev/rules.d/autonav.rules

# Reload udev
sudo service udev reload
sleep 2
sudo service udev restart

# Copy services
sudo cp autonav.service /etc/systemd/system/autonav.service
sudo cp autonav_service.sh /usr/bin/autonav_service.sh

# chmod time :D
sudo chmod +x /usr/bin/autonav_service.sh
sudo chmod 644 /etc/systemd/system/autonav.service

# boot time
echo "Use 'systemctl enable autonav' to enable the service at boot time."