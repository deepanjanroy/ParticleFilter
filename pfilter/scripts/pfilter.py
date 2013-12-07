#!/usr/bin/env python

import sys
import cv2
import random
import math

import roslib
roslib.load_manifest('pfilter')
import rospy
import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from pfilter_exceptions import ImageLoadError
import plotutil as ptl
import mathutil


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
        laser_range = (r[minus90index] * 10, r[zeroindex] * 10, r[plus90index] * 10)

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

    def dev_show_top_50(self):
        if self.weights:
            top = sorted(self.particles, key=lambda x: self.weights[x])[:50]
            for p in top:
                ptl.highlight_particle(self.map, p)

    def dev_highlight_all(self):
        for p in self.particles:
            ptl.highlight_particle(self.map, p)

    def draw_weights(self):
        if not self.weights:
            return
        font = cv2.FONT_HERSHEY_PLAIN
        col = 0
        scale = 0.8
        for p in self.particles:
            # t = str(self.weights[p])
            t = "%.4f" % self.weights[p]
            org = (int(p[1]), int(p[0]))
            cv2.putText(self.map, t, org, font, scale, col)

    def dev_user_particle(self):
        if self.loop_c != 0:
            self.loop_c = (self.loop_c + 1) % self.loop_mod
            return

        inp = raw_input("Enter new particle: ")
        if inp == "p":
            self.particles.pop()
        str_particle = inp.split(',')

        if inp == "pdb":
            import pdb; pdb.set_trace()


        if len(str_particle)==2 and str_particle[0] == "l":
            self.loop_c += 1
            self.loop_mod = int(str_particle[1])
            return

        if len(str_particle)==3:
            particle = tuple([float(x) for x in str_particle])
            self.particles.append(particle)
            self.num_particles += 1

    def check_convergence(self):
        self.convergence_counter = (self.convergence_counter + 1) % self.conv_check_rate
        if self.convergence_counter == 0:
            p = self.particles[0]
            for q in self.particles:
                if q[:2]!=p[:2]:
                    return
            self.angular_noise = 0




    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

            delta = self.calculate_delta()
            self.propagate(*delta)
            if self.user_particle_inserter: self.dev_user_particle()
            self.update_weights()

            self.check_convergence()

            self.clear_particles()
            if self.highlight_top: self.dev_show_top_50()
            if self.highlight_all: self.dev_highlight_all()
            if self.should_draw_weights: self.draw_weights()
            self.plot_all_particles()

            if not self.skip_resampling: self.resample(delta)
            cv2.imshow("Particle filter", self.map)
            k = cv2.waitKey(1)
            if k == 1048688:
                import pdb; pdb.set_trace()
            r.sleep()

    def draw_borders(self):
        nrows, ncols = self.map.shape
        for j in xrange(ncols):
            self.map.itemset((0,j), 0)
            self.map.itemset((nrows - 1, j), 0)

        for i in xrange(nrows):
            self.map.itemset((i, 0), 0)
            self.map.itemset((i, ncols - 1), 0)

    def mouse_callback(self, event, x, y, flag, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            ps = filter(lambda p: y-3< int(p[0]) < y+3 and x-3 < p[1] < x+3, self.particles)
            for p in ps:
                print "{0}: {1}".format(p, self.weights[p])

    def setup_callbacks(self):
        cv2.setMouseCallback("Particle filter", self.mouse_callback)
        self.loop_c = 0
        self.loop_mod = 10

    def __init__(self, map_path, **pf_settings):

        self.map = cv2.imread(map_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        if self.map == None:
            raise ImageLoadError

        self.draw_borders()
        self.map_clean = self.map.copy()
        cv2.namedWindow("Particle filter")

        rospy.init_node('particle_filter', anonymous=True)
        self.register_listeners()

        self.__dict__.update(pf_settings)

        self.last_processed_state = None
        self.last_received_state = None
        self.last_laser_state = None
        self.tr = ptl.Transformer(self.stage_x, self.stage_y, *(self.map.shape))

        self.num_old_particles = int((1 - self.random_restart) * self.num_particles)
        self.num_new_particles = self.num_particles - self.num_old_particles
        self.initialize_particles()
        self.resample_count = 0

        if self.use_mouse_callbacks: self.setup_callbacks()
        self.random_counter = 1
        self.convergence_counter = 1

    def new_particle(self):
        x = random.randint(0, self.map.shape[1] - 1)
        y = random.randint(0, self.map.shape[0] - 1)
        h = random.uniform(-math.pi, math.pi)

        p = (x, y, h)
        if ptl.in_free_space(self.map, x, y):
            return p
        else:
            return self.new_particle()

    def initialize_particles(self):

        self.particles = []
        self.weights = {}
        for i in xrange(self.num_particles):
            self.particles.append(self.new_particle())

    def plot_particle(self, particle):
        try:
            if not (particle[0] < 0 or particle[1] < 0):
                # self.map[particle[0], particle[1]] = random.randint(0,255)
                self.map[particle[0], particle[1]] = 0
        except IndexError:
            return

    def plot_particle_with_direction(self, particle):
        ptl.draw_direction(self.map, particle)
        self.plot_particle(particle)

    def plot_all_particles(self):
        for particle in self.particles:
            self.plot_particle(particle)
            if self.draw_direction:
                ptl.draw_direction(self.map, particle)

    def clear_particles(self):
        """
            Clears particles from the map. Not from the particle list
        """
        self.map = self.map_clean.copy()


    def normalize_weights(self):
        """
            Makes sure that the weights more or less sum up to 1
        """

        weight_sum = sum(self.weights.values())
        if weight_sum != 0:
            self.weights = { k : (self.weights[k] / weight_sum ) for k in self.weights}
        else:
            avg_weight = 1.0 / self.num_particles
            self.weights = { k : avg_weight for k in self.weights}

    def resample_old_particles(self):
        cumsum_weights = []

        cum_sum = 0
        for particle in self.particles:
            cum_sum += self.weights[particle]
            cumsum_weights.append(cum_sum)

        t = [random.uniform(0,1) for i in xrange(self.num_old_particles)]
        T = sorted(t)
        T.append(1)

        i,j = 0,0
        new_particles = []
        n = self.angular_noise

        while i < self.num_old_particles:
            if T[i] < cumsum_weights[j]:
                p = list(self.particles[j])
                noise = random.uniform(-n,n)
                p[2] = ptl.add_angle(p[2], noise)
                new_particles.append(tuple(p))
                i += 1
            else:
                j += 1

        return new_particles

    def generate_random_restarts(self):
        print "generating random dudes"
        self.random_counter = (self.random_counter + 1) % self.die_out_freq
        if self.random_counter == 0:
            print "Time to die"
            self.random_restart -= self.die_out_rate
            self.num_old_particles = int((1 - self.random_restart) * self.num_particles)
            self.num_new_particles = self.num_particles - self.num_old_particles
        return [self.new_particle() for i in xrange(self.num_new_particles)]

    def resample(self, delta):
        if not self.weights:
            return

        if delta == (0,0,0):
            return

        self.resample_count = (self.resample_count + 1) % self.resampling_rate
        if delta[2] > 0:
            self.particles = self.resample_old_particles() + self.generate_random_restarts()
            return

        if self.resample_count != 0:
            return

        self.particles = self.resample_old_particles() + self.generate_random_restarts()

    ## Find a way to kill particles that crash.
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

        if self.eval_degree == 3:
            for particle in self.particles:
                d_m90 = ptl.get_closest_obstacle(self.map_clean, particle, max_range=300, angle_off=m_ninety)
                d_0 = ptl.get_closest_obstacle(self.map_clean, particle, max_range=300, angle_off=0)
                d_90 = ptl.get_closest_obstacle(self.map_clean, particle, max_range=300, angle_off=ninety)

                diffs = (d_m90 - laser[0], d_0 - laser [1], d_90 - laser[2])
                # mean_diff = sum(diffs) / 3.0
                mean_diff = 0.15*diffs[0] + 0.7*diffs[1] + 0.15*diffs[2]

                if not ptl.in_free_space(self.map_clean, particle[0], particle[1]):
                    self.weights[particle] = 0
                else:
                    self.weights[particle] = mathutil.gaussian(mean_diff, sigma_sq = self.weight_stdev)

        if self.eval_degree == 1:
            for particle in self.particles:
                d_0 = ptl.get_closest_obstacle(self.map_clean, particle, max_range=300, angle_off=0)
                mean_diff = d_0 - laser[1]
                if not ptl.in_free_space(self.map_clean, particle[0], particle[1]):
                    self.weights[particle] = 0
                else:
                    self.weights[particle] = mathutil.gaussian(mean_diff, sigma_sq = self.weight_stdev, scale=1)

        if self.normalize: self.normalize_weights()


def main():
    args = sys.argv
    usage = "Usage = rosrun pfilter pfilter.py [png map image path]"

    if len(args) != 2:
        print usage
        return

    try:
        pf_settings = {
            'num_particles': 1000,
            'eval_degree': 3,
            'random_restart': 0.05,
            'weight_stdev': 1500,
            'resampling_rate': 10,
            'stage_x': 50,
            'stage_y': 50,
            'draw_direction': True,
            'skip_resampling': False,
            'should_draw_weights': True,
            'highlight_all': True,
            'highlight_top': False,
            'normalize': True,
            'use_mouse_callbacks': True,
            'user_particle_inserter': False,
            'die_out_rate': 0.1,
            'die_out_freq': 5,
            'conv_check_rate': 50,
            'angular_noise': 3.0
        }

        pf = ParticleFilter(args[1], **pf_settings)
    except ImageLoadError:
        print "Error loading map bitmap from " + args[1]
        return

    pf.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass





