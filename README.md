iowarrior
=========
This ros node provides access to the iowarrior device.
You can switch the states by calling internal services.
This package was created for ROS Hydro.

#Usage
After compiling the package you can run the node directly. 
A private parameter determines the device which should be controlled

* rosrun iowarrior iowarrior_node _device:=/dev/usb/iowarrior0

#Problemshooting
If you can not open the device, try to load the module.

* sudo modprobe iowarrior

Also the device file needs root permissions, but you can allow the user to open
the device too.

* sudo chmod 666 /dev/usb/iowarrior*

This has to be done each time you connect the warrior to your computer.
Alternatively you can copy the provided udev rule to /etc/udev/rules.d so you 
doesn't have to set the permissions everytime.
