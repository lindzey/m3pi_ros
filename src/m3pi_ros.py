#! /usr/bin/python

# ROS node that wraps the m3pi class for sending Twist commands to the robot
# Continually executes the last command sent ... 

import rospy
from geometry_msgs.msg import Twist

import m3pi

class m3pi_ros:
    def __init__(self):
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        if rospy.has_param('~port'):
            self.port = rospy.get_param('~port')
        else:
            rospy.logerr('m3pi_ros: ~port parameter must be set')
            raise Exception('m3pi_ros: ~port parameter must be set')

        self.robot = m3pi.m3pi()
        self.robot.connect(self.port)

        self.cmd_topic = rospy.get_param('~cmd_topic', 'cmd_vel')
        self.sub = rospy.Subscriber(self.cmd_topic, Twist, self.cmd_vel_cb)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def timer_cb(self, event):
        self.robot.set_vels(self.lin_vel, self.ang_vel)

    def cmd_vel_cb(self, msg):
        self.lin_vel = msg.linear.x
        self.ang_vel = msg.angular.z


if __name__=="__main__":
    rospy.init_node('m3pi_ros')
    myrobot = m3pi_ros()
    rospy.spin()
            
