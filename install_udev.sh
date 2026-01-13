#!/bin/bash

echo "Tianracer Udev Rules Updating..."
echo "Check it using the command : ls -l /dev | grep tianbot"

sudo rm -f /etc/udev/rules.d/99-tianbot-base.rules \
           /etc/udev/rules.d/99-tianbot-gps.rules \
           /etc/udev/rules.d/99-tianbot-rplidar.rules \
           /etc/udev/rules.d/99-tianbot-joystick.rules \
           /etc/udev/rules.d/99-tianbot-usb-cam.rules \
           /etc/udev/rules.d/99-tianbot-tom.rules

sudo cp ./_udev_/*.rules  /etc/udev/rules.d

echo " "
echo "Reloading udev rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "finished. "
