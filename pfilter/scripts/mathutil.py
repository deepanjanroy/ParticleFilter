import math

def gaussian(x, sigma_sq=500):
    return math.exp(-float(x)**2 / sigma_sq)
