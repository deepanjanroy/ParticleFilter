#!/usr/bin/env python

import cv2
import roslib; roslib.load_manifest('pfilter')
import rospy
import message_filters
import sys
# from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Dummy:

    # Problem Statement:
    # Your robot turns, and you have access to new_pose and end_pose. Simulate the same
    # pose change on each of the particles.

    # Problem:
    # Cache the last processes position, and update the state delta upon callback

    other_particle = (50,50,1.2)

    old_robot = (100, 100, .5)
    new_robot = (133, 142, .6)

    def laser_callback(self, data):
        pass

    def sync_callback(self, odom, laser):
        r = laser.ranges
        minus90 = 901
        zeroindex = 540
        plus90 = 181
        print "{0}:{1}:{2}".format(r[minus90], r[len(r) / 2], r[plus90])



    # def listener(self):
    #     rospy.init_node('listener', anonymous=True)
    #     rospy.Subscriber("odom", Odometry, self.odom_callback)
        # rospy.spin()

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        odom_sub = message_filters.Subscriber('odom', Odometry)
        laser_sub = message_filters.Subscriber('base_scan', LaserScan)
        ts = message_filters.TimeSynchronizer([odom_sub, laser_sub], 10)
        ts.registerCallback(self.sync_callback)
        rospy.spin()

    def showimage(self):
        cv2.imshow("qwe", self.im)
        cv2.waitKey(1)


    def __init__(self):
        self.w = cv2.namedWindow("qwe")
        self.im = cv2.imread("/home/2012/droy30/Worlds/minima.png", cv2.CV_LOAD_IMAGE_GRAYSCALE)
        self.heading = None


if __name__ == "__main__":
    c = Dummy()
    c.listener()
    while not rospy.is_shutdown():
        try:
            c.showimage()
            # c.waitKey
        except KeyboardInterrupt:
            sys.exit()
