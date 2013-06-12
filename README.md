m3pi_ros
========

Work-in-progress to interface the Pololu m3pi robot into my multirobot testbed.

m3pi_example.py - interactive python program to control m3pi wirelessly based on keyboard input. Written by Laura Lindzey and Ian Whitlock. To use:
* Install pyserial
* Plug in Wixel, note the serial port (on mac, it'll be something like '/dev/cu.usbmodemxxx'; Do 'ls -l /dev/*' and look for recently modified files)
* turn on Robot
* modify the serial variable in the python script
* run "python m3pi_example.py"
* type commands into terminal. Format is:
    linear(m/s), angular(rad/s). 
Max values achievable by the hardware are about +-1 m/s, and +-3.8rad/sec. A good starting point is '0.1, 0'

TODO:
* add command to terminate loop and close serial connection
* add command-line specification of serial port
* add control of LCD and speaker