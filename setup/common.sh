sudo apt-get update

# Copy the udev rules to the correct location
cp autonav.rules /etc/udev/rules.d/autonav.rules

# UDev rules are not reloaded automatically, so we need to do it manually
service udev reload
sleep 2
service udev restart

# Pip :)
sudo apt-get install python3-pip git -y
pip3 install -r requirements.txt