#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  telecoV"
echo "telecoV usb connection as /dev/telecoV , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy telecoV.rules to  /etc/udev/telecoV.d/"
echo "`rospack find telecoV`/scripts/telecoV.rules"
sudo cp `rospack find telecoV`/scripts/telecoV.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
