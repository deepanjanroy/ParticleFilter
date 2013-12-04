#!/usr/bin/env python

import roslib
roslib.load_manifest('pfilter')
import rospy
import sys
import cv2
import message_filters
import random
import math
import mathutil

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from pfilter_exceptions import ImageLoadError
import plotutil as ptl


class ParticleFilter:
    """
        Particles are defined to be a 3-tuple: (x, y, heading)
    """

    def sync_callback(self, odom, laser):
        stage_x = odom.pose.pose.position.x
        stage_y = odom.pose.pose.position.y
        stage_orientation = odom.pose.pose.orientation
        odom = self.tr.stage_to_cv((stage_x, stage_y,
                                         ptl.heading_from_qt(stage_orientation)))

        minus90index = 901
        zeroindex = 540
        plus90index = 181

        r = laser.ranges
        laser_range = (r[minus90index], r[zeroindex], r[plus90index])

        self.last_received_state = (odom, laser_range)

    def register_listeners(self):
        odom_sub = message_filters.Subscriber('odom', Odometry)
        laser_sub = message_filters.Subscriber('base_scan', LaserScan)
        ts = message_filters.TimeSynchronizer([odom_sub, laser_sub], 1)
        ts.registerCallback(self.sync_callback)

    def calculate_delta(self):
        if self.last_received_state == None:
            return (0, 0, 0)

        if self.last_processed_state == None:
            self.last_processed_state = self.last_received_state
            return (0, 0, 0)


        # Copying over the variables so that the callback threads
        # can't change them midway through calculations.
        # Note that last_rec_state = (odom, laser)
        laser = self.last_received_state[1]
        rec = self.last_received_state[0]
        proc = self.last_processed_state[0]

        dx = rec[0] - proc[0]
        dy = rec[1] - proc[1]
        dh = ptl.add_angle(rec[2], - proc[2])

        self.last_processed_state = (rec, laser)
        return (dx, dy, dh)

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # self.resample()

            self.clear_particles()
            self.plot_all_particles()

            delta = self.calculate_delta()
            self.propagate(*delta)
            self.update_weights()

            ## dev
            if self.weights:
                top = sorted(self.particles, key=lambda x: self.weights[x])[:50]
                for p in top:
                    ptl.highlight_particle(self.map, p)

            cv2.imshow("Particle filter", self.map)
            cv2.waitKey(1)
            r.sleep()

    def __init__(self, map_path, num_particles, eval_degree, stage_x=50, stage_y=50):

        self.map = cv2.imread(map_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        if self.map == None:
            raise ImageLoadError


        self.map_clean = self.map.copy()
        cv2.namedWindow("Particle filter")

        rospy.init_node('particle_filter', anonymous=True)
        self.register_listeners()


        self.last_processed_state = None
        self.last_received_state = None
        self.last_laser_state = None
        self.tr = ptl.Transformer(stage_x, stage_y, *(self.map.shape))

        self.num_particles = num_particles
        self.initialize_particles()

    def initialize_particles(self):

        def new_particle(self):
            x = random.randint(0, self.map.shape[1] - 1)
            y = random.randint(0, self.map.shape[0] - 1)
            h = random.uniform(-math.pi, math.pi)

            p = (x, y, h)
            if ptl.in_free_space(self.map, x, y):
                return p
            else:
                return new_particle(self)

        self.particles = []
        for i in xrange(self.num_particles):
            self.particles.append(new_particle(self))
        self.weights = {}

    def plot_particle(self, particle):
        try:
            if not (particle[0] < 0 or particle[1] < 0):
                self.map[particle[0], particle[1]] = random.randint(0,255)
        except IndexError:
            return

    def plot_particle_with_direction(self, particle):
        ptl.draw_direction(self.map, particle)
        self.plot_particle(particle)

    def plot_all_particles(self):
        for particle in self.particles:
            self.plot_particle_with_direction(particle)

    def clear_particles(self):
        """
            Clears particles from the map. Not from the particle list
        """
        self.map = self.map_clean.copy()

    def replot(self):
        pass

    def resample(self):
        pass

    def propagate(self, dx, dy, dh):
        for i,p in enumerate(self.particles):
            x = p[0] + dx
            y = p[1] + dy
            h = ptl.add_angle(p[2], dh)

            self.particles[i] = (x,y,h)

    def update_weights(self):
        if self.last_processed_state is None:
            return

        laser = self.last_processed_state[1]
        ninety = math.pi / 2
        m_ninety = - math.pi / 2

        self.weights = {}
        for particle in self.particles:
            d_m90 = ptl.get_closest_obstacle(self.map_clean, particle, max_range=300, angle_off=m_ninety)
            d_0 = ptl.get_closest_obstacle(self.map_clean, particle, max_range=300, angle_off=0)
            d_90 = ptl.get_closest_obstacle(self.map_clean, particle, max_range=300, angle_off=ninety)

            diffs = (d_m90 - laser[0], d_0 - laser [1], d_90 - laser[2])
            mean_diff = sum(diffs) / 3.0
            self.weights[particle] = mathutil.gaussian(mean_diff, sigma_sq = 1000)


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
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass





