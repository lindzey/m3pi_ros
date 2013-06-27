#! /usr/bin/python

import rospy
import tf
from numpy import mean

class worldPublisher():
    def __init__(self):
        # what the kinect is publishing tag locations in
        self.parent_frame = '/camera_rgb_optical_frame'
        # center of the published tags; will become world frame"
        self.world_frame = '/world'

        tag_ids = [5, 10, 15, 20, 25]
        self.tag_frames = ['/ar_marker_'+str(id) for id in tag_ids]

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.world_trans = None
        self.world_rot = None
        self.can_transform = False

    def find_frame(self):
        ground_xs = []
        ground_ys = []
        ground_zs = []
        for tag_frame in self.tag_frames:
            try:
                self.tf_listener.waitForTransform(self.parent_frame, tag_frame, rospy.Time(), rospy.Duration(5.0))
                (trans, rot) = self.tf_listener.lookupTransform(self.parent_frame, tag_frame, rospy.Time(0))
                if self.world_rot is None:
                    self.world_rot = rot
                ground_xs.append(trans[0])
                ground_ys.append(trans[1])
                ground_zs.append(trans[2])
                print "successful transform to ", tag_frame

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "failed to find camera -> tag transform", self.parent_frame, ' -> ', tag_frame

        if len(ground_xs) != 0:
            self.world_trans = [mean(ground_xs), mean(ground_ys), mean(ground_zs)]
            self.can_transform = True

    def pub_frame(self):
        if not self.can_transform:
            print "Did not initialize world frame! try again!"
            return

        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.tf_broadcaster.sendTransform(self.world_trans, self.world_rot, rospy.Time.now(), self.world_frame, self.parent_frame)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node('pub_world_frame')
    mypub = worldPublisher()
    mypub.find_frame()
    mypub.pub_frame()
    rospy.spin()
