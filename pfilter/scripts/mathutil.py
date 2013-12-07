import math

def gaussian(x, sigma_sq=500, scale=1):
    return scale * math.exp(-float(x)**2 / sigma_sq)
