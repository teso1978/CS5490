#!/bin/bash

# Raspberry Pi 
git clone git://git.drogon.net/wiringPi 
cd ./wiringPi 
./build 
cd ../ 
node-gyp rebuild 


echo '>>> Enable UART'
sudo raspi-config nonint do_serial 2

#if grep --quiet "^Revision\s*:\s*[ 123][0-9a-fA-F][0-9a-fA-F][0-9a-fA-F]0[8d][0-9a-fA-F]$" /proc/cpuinfo; then
   echo '>>> Disable bluetooth and enable PL011 uart'
   sudo raspi-config nonint set_config_var dtoverlay pi3-disable-bt /boot/config.txt
   sudo systemctl disable hciuart
#fi

echo '>>> CS5490 is installed.  Please reboot to apply changes.'