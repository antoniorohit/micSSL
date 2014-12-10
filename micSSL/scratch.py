import numpy as np
from scipy import optimize

DISTANCE_MIC = 0.20             # mt
r = 0.5773502*DISTANCE_MIC     # r = l/sqrt(3)
R = 0.8                         #    mt (Radius of Table)
K1 = r*r + R*R
K2 = 2*r*R
D = 0.1

def f(theta):
    y = D - np.sqrt(K1 - K2*np.cos(theta + 2*np.pi/3)) + np.sqrt(K1 - K2*np.cos(theta))
    return y


print optimize.newton(f,  .01)