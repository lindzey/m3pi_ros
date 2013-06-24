import math
import unittest

import m3pi_sim

# Stupid-simple test cases, checking that I understand how constructors
# work in python with optional arguments
# I feel wrong using "assertEquals" with doubles ... does unittest 
# transparently handle this similarly to gtest's "assertDoubleEqual"?
class TestM3piInit(unittest.TestCase):
    def testDefault(self):
        self.robot = m3pi_sim.m3pi_sim()
        self.assertEqual(0.0, self.robot.xx)
        self.assertEqual(0.0, self.robot.yy)
        self.assertEqual(0.0, self.robot.th)

    def testThOnly(self):
        th = math.pi/2
        self.robot = m3pi_sim.m3pi_sim(th=th)
        self.assertEqual(0.0, self.robot.get_x_pos())
        self.assertEqual(th, self.robot.get_th_pos())
 
    def testXxOnly(self):
        xx = 3.5
        self.robot = m3pi_sim.m3pi_sim(xx)
        self.assertEqual(xx, self.robot.get_x_pos())
        self.assertEqual(0.0, self.robot.get_y_pos())

    def testSetAll(self):
        xx = 3.5
        yy = 5.5
        th = math.pi/3
        self.robot = m3pi_sim.m3pi_sim(xx, yy, th)
        self.assertEqual(xx, self.robot.get_x_pos())
        self.assertEqual(yy, self.robot.get_y_pos())
        self.assertEqual(th, self.robot.get_th_pos())

# Checking that the update position functions behave as expected
class TestM3piStep(unittest.TestCase):
    def setUp(self):
        self.robot = m3pi_sim.m3pi_sim()

    def testForwards(self):
        self.assertEqual(0.0, self.robot.get_x_pos())
        self.assertEqual(0.0, self.robot.get_y_pos())
        self.assertEqual(0.0, self.robot.get_th_pos())
        
        lv = 1.0; #linear velocity
        av = 0.0; # angular velocity
        self.robot.set_vels(lv, av)
        self.robot.step_sim()
        # This relies on dt being hardcoded in to 0.05 ... BAD
        self.assertEqual(0.05, self.robot.get_x_pos())
        self.assertEqual(0.0, self.robot.get_y_pos())
        self.assertEqual(0.0, self.robot.get_th_pos())

    def testTurn(self):
        lv = 1.0; #linear velocity
        av = 0.0; # angular velocity
        self.robot.set_vels(lv, av)
        self.robot.step_sim()

        self.robot.set_vels(0.0, -1.0)
        self.robot.step_sim()

        self.assertEqual(0.05, self.robot.get_x_pos())
        self.assertEqual(0.0, self.robot.get_y_pos())
        self.assertEqual(-0.05, self.robot.get_th_pos())
        

        

if __name__=="__main__":
    unittest.main()
         
