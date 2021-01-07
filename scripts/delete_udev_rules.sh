#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to cyglidar"
echo "sudo rm   /etc/udev/rules.d/cyglidar.rules"
sudo rm   /etc/udev/rules.d/cyglidar.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
