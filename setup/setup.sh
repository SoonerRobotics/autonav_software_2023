#!/bin/bash

bash common.sh

# Copy services
sudo cp autonav.service /etc/systemd/system/autonav.service
sudo cp autonav_service.sh /usr/bin/autonav_service.sh

# chmod time :D
sudo chmod +x /usr/bin/autonav_service.sh
sudo chmod 644 /etc/systemd/system/autonav.service

# boot time
echo "Use 'systemctl enable autonav' to enable the service at boot time."