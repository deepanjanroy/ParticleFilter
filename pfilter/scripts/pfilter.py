import roslib
roslib.load_manifest('pfilter')
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

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
    


    def __init__(map_path, num_particles, eval_degree):
        pass

    def spin():
        pass
        # The main loop

    



if __name__ == "__main__": 
    pf = ParticleFilter()
    pf.spin()

    c.listener()
    while not rospy.is_shutdown():
        try:
            print c.heading
            c.showimage()
            # c.waitKey
        except KeyboardInterrupt:
            print "Bye"
            sys.exit()

