#!/usr/bin/env python3

from numpy import *
import numpy as np

def spectral_radius(Ann: array) -> float64:
    Ann_dim = Ann.shape
    if len(Ann_dim) != 2 :
        print("Error: Input A is not a matrix!")
        return

    if Ann_dim[0] != Ann_dim[1] :
        print("Error: Input A is not a square matrix!")
        return

    try :
        e = np.linalg.eig(Ann)
        spr = np.max(np.abs(e[0]))
        return spr
    except :
        pass
