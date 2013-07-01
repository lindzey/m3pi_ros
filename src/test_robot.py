#! /usr/bin/python

import math
import unittest

import rospy


import tf
from geometry_msgs.msg import Twist

class robot():
    def __init__(self):
        rospy.init_node('test_robot', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher('/cmd_vel', Twist)
        # Make sure robot is stopped
        self.pub.publish(Twist())
        # These should really be parameters set in the launch file
        self.world_frame = '/world'

        #self.robot_frame = '/m3pi'
        self.robot_frame = '/ar_marker_0'
        # TODO: wait for transform here? (need pub_world_frame to finish....)

    def get_pos(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return (None, None, None)
        xx = trans[0]
        yy = trans[1]
        th = tf.transformations.euler_from_quaternion(rot)[2]
        return (xx, yy, th)

    def drive(self, vel, dist):
         # Command motion
         dt = abs(dist / vel)
         msg = Twist()
         msg.linear.x = vel
         self.pub.publish(msg)
         rospy.sleep(dt)
         self.pub.publish(Twist())

    def turn(self, omega, theta):
        # Command motion
        dt = abs(theta / omega)
        msg = Twist()
        msg.angular.z = omega
        self.pub.publish(msg)
        rospy.sleep(dt)
        self.pub.publish(Twist())
        

class TestRobotMotion(unittest.TestCase):
    def setUp(self):
        self.robot = robot()
        self.ang_tolerance = math.pi/4 # angle...
        self.pos_tolerance = 0.2 # percent offset

    def test_forward_slow(self):
        """ Test robot moving forward at 0.3 m/s for 1 sec"""
        raw_input("Press enter when robot at center of frame.")
        cmd_vel = 0.3
        cmd_dist = 0.3

        pos1 = self.robot.get_pos()
        # move forward at 0.3 m/s for 0.3 m
        self.robot.drive(cmd_vel, cmd_dist)
        pos2 = self.robot.get_pos()
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        dist = math.sqrt(dx*dx + dy*dy)
        th_delta = math.atan2(dy, dx)
        th_robot = 0.5*(pos2[2] + pos1[2])
        # find difference between robot's actual heading and the delta traveled
        # this will be close to 1 if they're the same angle, and close to -1 if they're off by pi
        self.assertGreater(math.cos(th_delta - th_robot), math.cos(self.ang_tolerance))
        self.assertLess(abs(dist - cmd_dist), self.pos_tolerance * cmd_dist)
        
        


if __name__ == "__main__":
    unittest.main()
