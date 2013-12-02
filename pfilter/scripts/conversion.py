import roslib
roslib.load_manifest('tf')
import tf
import math

def heading_from_qt(q):
    preshifted =  tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
    shifted = preshifted + (math.pi / 2)
    if shifted >= math.pi:
        return shifted - 2 * math.pi
    else:
        return shifted

