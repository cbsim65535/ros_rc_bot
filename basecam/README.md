#readme#

## pip install
pip3 install pylink
pip3 install transforms3d

## initial usb mcu ##
$ lsusb

Bus 001 Device 003: ID 10c4:ea60 
Cygnal Integrated Products, Inc. CP210x UART Bridge / myAVR mySmartUSB light

$ udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -n1

ATTRS{serial}=="0001"

$ sudo vi /etc/udev/rules.d/basecam.rules

SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttySBGC"

$ sudo adduser pi dialout

$ sudo reboot
