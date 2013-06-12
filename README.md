m3pi_ros
========

Work-in-progress to interface the Pololu m3pi robot into my multirobot testbed.

m3pi_example.py - interactive python program to control m3pi wirelessly based on keyboard input. Written by Laura Lindzey and Ian Whitlock. To use:
* Install pyserial
* Plug in Wixel, note the serial port (on mac, '/dev/cu.usbmodemxxx')
* turn on Robot
* modify the serial variable in the python script
* run "python m3pi_example.py"
* type commands into terminal. Format is:
    linear, angular