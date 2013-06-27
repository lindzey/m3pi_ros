#! /usr/bin/python
import math
import rospy

import tf

from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist 

class simpleController():

    def __init__(self):
        self.has_goal = False;
        self.goal_x = 0.0
        self.goal_y = 0.0
        # Used for looking up which marker is the goal based on color
        self.color_lookup = {'red':20, 'yellow':15, 'green':25, 'blue':10, 'purple':5}
        self.world_frame = 'world'
        #self.robot_frame = 'm3pi'
        self.robot_frame = '/ar_marker_0'

        self.point_sub = rospy.Subscriber('/goal_pos', Point, self.point_cb)
        self.color_sub = rospy.Subscriber('/goal_color', String, self.color_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist)

        self.tf_listener = tf.TransformListener()

    def point_cb(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.has_goal = True
        print "received new goal position! ", self.goal_x, self.goal_y

    def color_cb(self, msg):
        try:
            goal_id = self.color_lookup[msg.data]
            print "received new goal color! ", msg.data, ", with ID: ", goal_id
        except (KeyError):
            goal_id = None
            print "Received goal color not in dict! ", msg.data
            return

        if goal_id is not None:
            goal_frame = '/ar_marker_' + str(goal_id)
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, goal_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "attempted to find world->color transform; failed."
                self.has_goal = False
                return
        
        self.has_goal = True
        self.goal_x = trans[0]
        self.goal_y = trans[1]


   
    def run(self):
        """ continually calculates and sends commands to the robot """
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            lin_vel = 0
            ang_vel = 0
            if self.has_goal:
                # TODO: add in calculation of errors and tf stuff here
                try:
                      (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                # TODO: calculate angular and linear angles ...
                robot_x = trans[0]
                robot_y = trans[1]
                euler = tf.transformations.euler_from_quaternion(rot)
                robot_th = euler[2]

                dx = self.goal_x - robot_x
                dy = self.goal_y - robot_y
                goal_th = math.atan2(dy, dx)
                dth = robot_th - goal_th
                while dth < -1.0*math.pi:
                    dth = dth + 2*math.pi
                while dth > 1.0*math.pi:
                    dth = dth - 2*math.pi

                dr = math.sqrt(dx*dx + dy*dy)
                if abs(dr) < 0.02:
                    print "Success! At goal!"
                    self.has_goal = False

                ang_vel = -1.0 * math.copysign(max(0.2, min(0.5, 0.4*abs(dth))), dth)
                if abs(dth) > math.pi/4:
                    lin_vel = 0.0
                else:
                    lin_vel = max(0.1, min(0.5, 0.5*dr))
                
            msg = Twist()
            msg.linear.x = lin_vel
            msg.angular.z = ang_vel
            self.cmd_pub.publish(msg)
            rate.sleep()



if __name__=="__main__":
    rospy.init_node('m3pi_controller', anonymous=True)
    controller = simpleController()
    controller.run()
    rospy.spin()
    
