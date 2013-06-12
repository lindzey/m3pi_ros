import serial

class m3pi:
        """Simple class to allow controlling the m3pi robot over a Wixel serial link"""

        # define command characters for the m3pi
        sig = '\x81'
        m1forward = '\xc1'
        m1back = '\xc2'
        m2forward = '\xc5'
        m2back = '\xc6'
        baud_rate = 115200

        def __init__(self):
                self.ser = None;

        def connect(self, serial_port):
                """ Attempts to connect to the Wixel located at serial_port, and ping
                the robot. If robot is not turned on, not connected, or does not return
                the correct serial response, this will raise an exception. """

                if self.ser is not None:
                        raise Excpetion("attempting to connect when already connected!")

                self.ser = serial.Serial(serial_port, self.baud_rate)
                self.ser.flush()
                # for testing if we can ping the robot, need to timeout in case of no response
                self.ser.timeout=500 
                self.ser.write(self.sig)
                response = self.ser.read(6)

                if response != '3pi1.1':
                        print response
                        raise Exception("bad response from robot")
                else:
                        print "success"

                # for everything else, we'll know what we expect to hear, and can wait
                # TODO: maybe it would be better to raise exceptions than just wait forever?
                self.ser.timeout=None
                
        def disconnect(self):
                """ Cleanly disconnects from the wixel's serial port """
                self.ser.close()
                self.ser = None

        def set_vels(self, linear, angular):
                """ Given the input linear and angular velocities (in m/s and rad/s), 
                generate and send the required motor commands to have the robot execute
                them. The robot is capable of approximately +- 1m/s, and +-3.8 rad/s.
                These maxima are assuming a commanded linear with no angular, or angular
                with no linear. 
                Input values greater than feasible will be silently thresholded"""

                motor1 = 127*(linear-angular/3.8)
                motor2 = 127*(angular/3.8+linear)
                
                # allowable motor inputs are 0-127
                if motor1 >= 0:
                        val = min(int(motor1), 127)
                        self.ser.write(self.m1forward)
                        self.ser.write(chr(val))
                else:
                        val = min(int(-1*motor1), 127)
                        self.ser.write(self.m1back)
                        self.ser.write(chr(val))
                        
                if motor2 >= 0:
                        val = min(int(motor2), 127)
                        self.ser.write(self.m2forward)
                        self.ser.write(chr(val))
                else:
                        val = min(int(-1*motor2), 127)
                        self.ser.write(self.m2back)
                        self.ser.write(chr(val))
                


                
                



