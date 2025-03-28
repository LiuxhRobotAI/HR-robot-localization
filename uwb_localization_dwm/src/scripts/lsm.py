#!/usr/bin/env python3

from numpy import *
import numpy as np

def lsm(Amn: np.ndarray, bm: np.ndarray) -> array: # -> matrix
    A = np.mat(Amn)
    bm=np.reshape(bm,(3,1))
    b = np.mat(bm)

    return (A.T * A).I * A.T *b

if __name__ == '__main__':

    Amn = np.mat([[1,0,0], [0,2,0], [0,0,3]]) # 3D
    dist_bm = np.mat([[1], [1], [1]])

    # x_ls = np.linalg.lstsq(Amn, dist_bm, rcond = -1) 
    # x_ls = np.linalg.pinv(Amn)
    x_ls = lsm(Amn, dist_bm)

    print(x_ls)
