#!/bin/bash

echo "◯ delete remap the device serial port(ttyUSBX) to  telecoV"
echo "sudo rm   /etc/udev/rules.d/telecoV.rules"
sudo rm   /etc/udev/rules.d/telecoV.rules
echo " "
echo "●︎ Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "○ finish  delete"
