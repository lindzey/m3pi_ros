import m3pi
import string

myrobot = m3pi.m3pi()

#myrobot.connect('/dev/cu.usbmodem1421')
myrobot.connect('/dev/ttyACM0')

print("enter desired linear/angular robot speeds, or 'q' to quit. Example: \n 0.2, 0.0\n")

try:
    input=raw_input()
    while(input != 'q'):
        vel_strs = string.split(input,",")
        if len(vel_strs) != 2 :
            print "usage: linear, angular"
        else:
            lin_vel = float(vel_strs[0])
            ang_vel = float(vel_strs[1])
            myrobot.set_vels(lin_vel, ang_vel)
        input = raw_input()
finally:
    myrobot.set_vels(0, 0)
    myrobot.disconnect()

