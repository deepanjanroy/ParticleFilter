import cv2
import roslib
roslib.load_manifest('tf')
import tf
import math

# Problem: The stage coordinates and the cv image coordinates do not coincide. Fix it. 
# For this assignment, we keep everything in cv coordinates. Propagation will need to be solved carefully.

def draw_direction(img, particle, length=10, thickness=1):
    """
        Draws a line at the direction of the heading of a 
        particle so that you can see it on the map.

        A particle is only a 3-tuple: (x, y, heading) - 
        everything in cv coordinates.
    """

    p1 = (particle[0], particle[1])
    angle = particle[2]
    p2_x = int( p1[0] + length * math.cos(angle) )
    p2_y = int( p1[1] + length * math.sin(angle) )
    p2 = (p2_x, p2_y)
    cv2.line(img, p1, p2, 0, 3)


def heading_from_qt(q):
    """
        Gets the Stage heading from the quaternions given in stage.
        Heading is in Stage coordinates - it increases counter-clockwise.
    """ 
    preshifted = tf.transformations.euler_from_quaternion(
                                      (q.x, q.y, q.z, q.w))[2]

    shifted = preshifted + (math.pi / 2)
    
    if shifted >= math.pi:
        return shifted - 2 * math.pi
    else:
        return shifted


class Transformer:

    def delta_stage_to_cv(self, state_delta):
        """
            Convert a particle position delta from stage 
            coordinates to opencv coordinates. This will
            really only work to convert deltas, because
            I disregard the different positions of the origin.

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

        dx = state_delta[0]
        dy = state_delta[1]
        dh = state_delta[2]

        cv_dx = - int (dy * self.cv_x_factor)
        cv_dy = int (dx * self.cv_y_factor)
        cv_dh = -dh

        return (cv_dx, cv_dy, cv_dh)

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
