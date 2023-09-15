""" catch-all script for utility data structures (structs) and functions """

from typing import List
from dataclasses import dataclass, field
import math
import numpy as np

@dataclass
class HBParams:
    """  
    This dataclass captures the basic parameters of the Hummingbird drone [in development]
    """

    I = np.array([[1.43e-5,   0,          0], 
                  [0,         1.43e-5,    0],
                  [0,         0,          2.89e-5]])
    invI = np.linalg.inv(I)

    mass:   float = 0.030
    I:      np.ndarray = I
    invI:   np.ndarray = invI
    g:      float = 9.81
    L:      float = 0.046
    max_angle: float = 40*math.pi/180
    maxT:   float = 2.5*mass*g
    minT:   float = 0.05*mass*g

    # matrix to compute individual rotor thrusts from U array
    A = np.array([[0.25,    0,     -0.5/L],
                  [0.25,    0.5/L,  0],
                  [0.25,    0,      0.5/L],
                  [0.25,    -0.5/L, 0]])
    
    # matrix to re-arrange the U array
    B = np.array([[1,   1,   1,  1],
                  [0,   L,   0, -L],
                  [-L,  0,   L,  0]])

def angle_wrap(angle : float):
    return (angle + np.pi) % (2 * np.pi) - np.pi
