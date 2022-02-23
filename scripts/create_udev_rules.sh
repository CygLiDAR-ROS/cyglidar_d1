#!/bin/bash

echo " "
echo "Remap the device serial port(ttyUSBX) to  cyglidar"
echo "Check if USB is identified as /dev/cyglidar using the command: ls -l /dev|grep ttyUSB"
echo "Start copying cyglidar.rules to  /etc/udev/rules.d/"
echo "`rospack find cyglidar_d1`/scripts/cyglidar.rules"
sudo cp `rospack find cyglidar_d1`/scripts/cyglidar.rules  /etc/udev/rules.d
echo " "
echo "Restart udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Created"
