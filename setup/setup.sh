#!/bin/bash

bash common.sh

# Copy services
cp autonav.service /etc/systemd/system/autonav.service
cp autonav_service.sh /usr/bin/autonav_service.sh

# chmod time :D
chmod +x /usr/bin/autonav_service.sh
chmod 644 /etc/systemd/system/autonav.service

# boot time
echo "Use 'systemctl enable autonav' to enable the service at boot time."