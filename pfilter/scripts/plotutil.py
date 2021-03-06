import cv2
import math

import roslib
roslib.load_manifest('tf')
import tf

"""
    Unless otherwise notes, all coordinates are in OpenCV coordinates."
"""

def get_closest_obstacle(img_clean, particle, max_range, angle_off=0):
    """
        Does what it's named.
        Make sure the img is clean - i.e. it has no particles.
    """

    lookup = img_clean.item
    x_length = img_clean.shape[1]
    y_length = img_clean.shape[0]

    px = particle[0]
    py = particle[1]
    angle = add_angle(particle[2], angle_off)

    x_comp = math.sin(angle)
    y_comp = math.cos(angle)

    for r in xrange(5, max_range):
        x = px + r * x_comp
        y = py + r * y_comp

        if x < 0 or y < 0 or x >= x_length or y >= y_length:
            return r

        if lookup((x,y)) == 0 :
            return r

    return max_range

def highlight_particle(img, particle):
    """
        Draws a small circle around the particle.
    """
    x = int (particle[0])
    y = int (particle[1])
    cv2.circle(img, (y, x), radius=3, color=0, thickness=1)

def draw_direction(img, particle, length=10, thickness=1):
    """
        Draws a line at the direction of the heading of a
        particle so that you can see it on the map.

        A particle is only a 3-tuple: (x, y, heading) -
        everything in cv coordinates.
    """

    p1 = (int (particle[1]), int(particle[0]))
    angle = particle[2]
    p2_x = int( p1[0] + length * math.cos(angle) )
    p2_y = int( p1[1] + length * math.sin(angle) )
    p2 = (p2_x, p2_y)
    try:
        # from IPython import embed; embed()
        cv2.line(img, p1, p2, color=0, thickness=thickness)
    except IndexError:
        if length < 3:
            return
        draw_direction(img, particle, length=float(length)/2)

def in_free_space(img, x, y):
    """
        Checks if a point has valid coordinates.
        Returns false if point coincides with an obstacle,
        or if the coordinates are out of bounds.
    """
    try:
        return img[x, y] != 0
    except IndexError:
        return False


def add_angle(angle_1, angle_2):
    """
        Adds two angles, but making sure the return value is
        between -pi and pi.
    """
    ret = angle_1 + angle_2

    if ret >= math.pi:
        return ret - 2 * math.pi
    elif ret <= -math.pi:
        return ret + 2 * math.pi
    else:
        return ret

def heading_from_qt(q):
    """
        Gets the Stage heading from the quaternions given in stage.
        Heading is in Stage coordinates - it increases counter-clockwise.
    """
    preshifted = tf.transformations.euler_from_quaternion(
                                      (q.x, q.y, q.z, q.w))[2]

    return add_angle(preshifted, math.pi / 2)

class Transformer:

    def stage_to_cv(self, state_delta, offset_x=0, offset_y=0):
        """
            Convert a particle position delta from stage
            coordinates to opencv coordinates. This will
            really only work properly to convert deltas, because
            I disregard the different positions of the origin.
            Returns floats for maximum information retention.

            I get heading in opencv by taking atan (y/x).
            This heading increases clockwise, while in stage
            coordinates, heading increases counter-clockwise.

            I hope this makes it clear why the x and y coordinates
            will change.

            |----------y        |-------------x
            |                   |
            |   OpenCV          |   Stage
            |                   |
            x                   -y

        """

        x = state_delta[0]
        y = state_delta[1]
        h = state_delta[2]

        cv_x = - y * self.cv_x_factor + offset_x
        cv_y = x * self.cv_y_factor + offset_y
        cv_h = -h

        return (cv_x, cv_y, cv_h)

    def __init__(self, stage_x_length, stage_y_length,
                          img_x_length, img_y_length):
        """
            Initializes a transformer object that caches the scaling
            information so that they do not need to be recalculated.

            The x factor will be multiplies to cv coordinates.
            Recall that opencv x coordinates actually go
            downwards.
        """

        self.cv_x_factor = float(img_x_length) / float(stage_y_length)
        self.cv_y_factor = float(img_y_length) / float(stage_x_length)


