#! /usr/bin/python

# ROS node that listens for geometry_msgs/Twist, and outputs the 
# position that the m3pi would be at, given the input commands

import rospy

import tf
from geometry_msgs.msg import Twist

import math

def test_cb(event):
    print "called cb!"

class m3pi_sim:
    
    def __init__(self):
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        # and, set up the ROS stuff

        init_pos = rospy.get_param('~init_pos', {'x':0.0, 'y':0.0, 'th':0.0})
        self.th = init_pos['th']
        self.xx = init_pos['x']
        self.yy = init_pos['y']

        self.dt = rospy.get_param('~dt', 0.05)
        self.robot_frame = rospy.get_param('~robot_frame', 'm3pi')
        self.parent_frame = rospy.get_param('~world_frame', 'world')
        self.cmd_topic = rospy.get_param('~cmd_topic', 'cmd_vel')

        # This needs to happen *AFTER* settin up the params/topics
        self.tfb = tf.TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_cb)
        self.sub = rospy.Subscriber(self.cmd_topic, Twist, self.cmd_vel_cb)



    def get_x_pos(self):
        return self.xx
    def get_y_pos(self):
        return self.yy
    def get_th_pos(self):
        return self.th

    def set_vels(self, lin_vel, ang_vel):
        """ Function that'll be called by the ROS cmd_vel callback """
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel

    def step_sim(self):
        """ Function that updates robot's position X = X + dt*(dX/dt) 
        Will be called by the timer callback to run at (1/dt) Hz"""
        self.th = self.th + self.dt * self.ang_vel
        self.xx = self.xx + self.dt * self.lin_vel * math.cos(self.th)
        self.yy = self.yy + self.dt * self.lin_vel * math.sin(self.th)

    def pub_tf(self):
        """ In charge of actually publishing teh robot's position.
        Assumes that the robot's frame is self.robot_name, and that we're
        using the World coordinates """
        self.tfb.sendTransform((self.xx, self.yy, 0.0), 
                               tf.transformations.quaternion_from_euler(0.0, 0.0, self.th), 
                               rospy.Time.now(), 
                               self.robot_frame, 
                               self.parent_frame)

    def timer_cb(self, event):
        self.step_sim()
        self.pub_tf()

    def cmd_vel_cb(self, msg):
        """ Updates internal velocities according to specified command. """
        print "recedived cmd_vel callback!"
        # can only travel along robot's local x-axis, and rotate about z-axis
        self.set_vels(msg.linear.x, msg.linear.z) 

if __name__=="__main__":
    # TODO: should this be anonymous for running multiples?
    rospy.init_node('m3pi_sim')
    mysim = m3pi_sim()
    rospy.spin()
