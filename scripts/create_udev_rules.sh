#!/bin/bash

echo " "
echo "remap the device serial port(ttyUSBX) to  cyglidar"
echo "Check if USB is identified as /dev/cyglidar using the command: ls -l /dev|grep ttyUSB"
echo "Start copying cyglidar.rules to  /etc/udev/rules.d/"
sudo cp `ros2 pkg prefix cyglidar_d1_ros2`/share/cyglidar_d1_ros2/scripts/cyglidar.rules  /etc/udev/rules.d
echo " "
echo "Restart udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Created"
