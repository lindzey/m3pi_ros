# worked on this with Ian Whitlock

import serial
import string

# define command characters for the m3pi
sig = '\x81'
m1forward = '\xc1'
m1back = '\xc2'
m2forward = '\xc5'
m2back = '\xc6'

ser = serial.Serial('/dev/cu.usbmodem1421',115200)
ser.flush()

ser.timeout=500
ser.write(sig)
response = ser.read(6)
if response != '3pi1.1':
        print response
        raise Exception("bad response from robot")
else:
        print "success"


while(True):
        input = raw_input()
        vel_strs = string.split(input,",")
        if len(vel_strs) != 2 :
                print "usage: linear, angular"
                continue
        
        lin_vel = float(vel_strs[0])
        ang_vel = float(vel_strs[1])

        motor1 = 127*(lin_vel-ang_vel/3.8)
        motor2 = 127*(ang_vel/3.8+lin_vel)
        
        # allowable motor inputs are 0-127
        if motor1 >= 0:
                val = min(int(motor1), 127)
                ser.write(m1forward)
                ser.write(chr(val))
        else:
                val = min(int(-1*motor1), 127)
                ser.write(m1back)
                ser.write(chr(val))

        if motor2 >= 0:
                val = min(int(motor2), 127)
                ser.write(m2forward)
                ser.write(chr(val))
        else:
                val = min(int(-1*motor2), 127)
                ser.write(m2back)
                ser.write(chr(val))

                
        
ser.close()	
