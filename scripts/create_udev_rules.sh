#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  cyglidar"
echo "ls01c usb cp210x connection as /dev/cyglidar , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy cyglidar.rules to  /etc/udev/rules.d/"
echo "`rospack find cld1a0_pcl`/scripts/cyglidar.rules"
sudo cp `rospack find cld1a0_pcl`/scripts/cyglidar.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
