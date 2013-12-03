import cv2

# Problem: The stage coordinates and the cv image coordinates do not coincide. Fix it. 
# For this assignment, we keep everything in cv coordinates. Propagation will need to be solved carefully.

def draw_direction(img, particle, length=10, thickness=1):
    p1 = (state[0], state[1])
    angle = state[2]
    p2_x = int( p1[0] + length * math.cos(angle) )
    p2_y = int( p1[1] + length * math.sin(angle) )

    cv2.line(im, p1, p2, 0, 3)
