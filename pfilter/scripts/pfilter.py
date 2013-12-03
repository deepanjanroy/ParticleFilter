#!/usr/bin/env python

import roslib
roslib.load_manifest('pfilter')
import rospy
import sys
import cv2
import message_filters
import random

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from pfilter_exceptions import ImageLoadError
import plotutil as ptl

class Particle:

    def __init__(self, x,y,h):
        self.x = x
        self.y = y
        self.h = h

    @property
    def tuple(self):
        return (self.x, self.y, self.h)

    def draw(self):
        pass


class ParticleFilter:

    def sync_callback(self, odom, laser):
        stage_x = odom.pose.pose.position.x
        stage_y = odom.pose.pose.position.y
        stage_orientation = odom.pose.pose.orientation
        self.last_received_odom_stage = (stage_x, stage_y,
                                         stage_orientation)

    def register_listeners(self):
        odom_sub = message_filters.Subscriber('odom', Odometry)
        laser_sub = message_filters.Subscriber('base_scan', LaserScan)
        ts = message_filters.TimeSynchronizer([odom_sub, laser_sub], 1)
        ts.registerCallback(self.sync_callback)

    def dev_process_data(self):
        # This function can be optimized. Right now it's written
        # for maximum readability, not maximum correctness.

        received_stage = self.last_received_odom_stage
        processed_stage = self.last_processed_odom_stage

        print received_stage
        print "Processed " + str(processed_stage)

        if received_stage == None:
            return
        if processed_stage == None:
            self.last_processed_odom_stage = received_stage
            return

        received = list(received_stage)
        processed = list(processed_stage)

        received[2] = ptl.heading_from_qt(received[2])
        processed[2] = ptl.heading_from_qt(processed[2])

        delta = [received[i] - processed[i] for i in xrange(3)]

        converted_delta = self.tr.delta_stage_to_cv(delta)

        self.last_processed_odom_stage = self.last_received_odom_stage
        self.last_known_state_cv = tuple(
                [self.last_known_state_cv[i] + converted_delta[i]
                for i in xrange(3)])



    def spin(self):
        while not rospy.is_shutdown():
            # self.propagate()
            # self.update()
            # self.resample()

            # Dev
            self.dev_process_data()
            self.plot_particle(self.last_known_state_cv)

            cv2.imshow("Particle filter", self.map)
            cv2.waitKey(1)

    def __init__(self, map_path, num_particles, eval_degree,
                        stage_x=50, stage_y=50):
        self.map = cv2.imread(map_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        if self.map == None:
            raise ImageLoadError

        cv2.namedWindow("Particle filter")

        rospy.init_node('particle_filter', anonymous=True)
        self.register_listeners()

        self.last_processed_odom_stage = None
        self.last_received_odom_stage = None
        self.last_known_state_cv = (100, 100, 0)
        self.tr = ptl.Transformer(stage_x, stage_y, *(self.map.shape))


    def plot_particle(self, particle):
        self.map[particle[0], particle[1]] = random.randint(0,255)
        ptl.draw_direction(self.map, particle)

    def plot_all_particles(self):
        pass

    def clear_particles(self):
        pass

    def replot(self):
        pass

    def resample(self):
        pass

    def propagate(self, dx, dy, dh):
        pass

    def update_weights(self):
        pass


def main():
    args = sys.argv
    usage = "Usage = rosrun pfilter pfilter.py [png map image path]"

    if len(args) != 2:
        print usage
        return

    try:
        pf = ParticleFilter(args[1], num_particles=1000, eval_degree=1)
    except ImageLoadError:
        print "Error loading map bitmap from " + args[1]
        return

    pf.spin()


if __name__ == "__main__":
    main()





