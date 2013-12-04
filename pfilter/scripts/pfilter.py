#!/usr/bin/env python

import roslib
roslib.load_manifest('pfilter')
import rospy
import sys
import cv2
import message_filters
import random
import math

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
        self.last_received_state = self.tr.stage_to_cv((stage_x, stage_y,
                                         ptl.heading_from_qt(stage_orientation)))

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
        rec = self.last_received_state
        proc = self.last_processed_state

        dx = rec[0] - proc[0]
        dy = rec[1] - proc[1]
        dh = ptl.add_angle(rec[2], - proc[2])

        self.last_processed_state = rec
        return (dx, dy, dh)

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # self.propagate()
            # self.update()
            # self.resample()

            # Dev
            # self.dev_process_data()
            self.clear_particles()
            self.plot_all_particles()

            delta = self.calculate_delta()
            self.propagate(*delta)
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
        for particle in self.particles:
            ptl.get_closest_obstacle(self.map_clean, particle, max_range=300)
            difference =


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





