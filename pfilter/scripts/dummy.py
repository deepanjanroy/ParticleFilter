#!/usr/bin/env python

import roslib; roslib.load_manifest('pfilter')
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry


def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" %data.header.stamp.secs)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback)
    rospy.spin()


if __name__ == "__main__": 
    listener()
