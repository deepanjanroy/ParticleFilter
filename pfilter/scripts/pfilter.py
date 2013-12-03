#!/usr/bin/env python

import roslib
roslib.load_manifest('pfilter')
import rospy
import sys
import cv2

from std_msgs.msg import String
from nav_msgs.msg import Odometry

from conversion import heading_from_qt
from pfilter_exceptions import ImageLoadError

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

    def sync_callback(self):
        pass
    
    def register_listeners(self):
        pass

    def __init__(self, map_path, num_particles, eval_degree):
        self.map = cv2.imread(map_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        if self.map == None:
            raise ImageLoadError

        cv2.namedWindow("Particle filter")

        rospy.init_node('particle_filter', anonymous=True)
        self.register_listeners()

    def spin(self):
        while not rospy.is_shutdown():
            self.propagate()
            self.update()
            self.resample()

            cv2.imshow("Particle filter", self.map)
            cv2.waitKey(1)

    def plot_particle(self, particle):
        pass

    def plot_all_particles(self):
        pass

    def clear_particles(self):
        pass

    def replot(self):
        pass

    def resample(self):
        pass

    def in_obstacle(self):
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





