#!/usr/bin/env python3

from numpy import *
import numpy as np
from spectral_radius import spectral_radius

def hrm(Amn: array, Rnn: array, k: 'int >= 0' = 0,
        w:'float64 >= 0.0' = 0.0, correctbias_w:'float64' = True) -> array:
    if (k < 0) :
        k = 0 # TR

    if (w < 0) or (w > 1) :
        w = 0

    try:
        correctbias_w
    except NameError:
        correctbias_w = False
    except:
        pass

    A = np.mat(Amn)
    R = np.mat(Rnn)
    AAR_inv = (A.T* A + R).I
    if spectral_radius(R * AAR_inv) is None or spectral_radius(R * AAR_inv) > 1 :
        Rnn = np.zeros(Rnn.shape)
        print("Error: the required spectral radius condition doesn't satisfy!")

    HRM_k = np.zeros(Rnn.shape)
    for i in range(0, k+1, 1) :
        HRM_k = HRM_k + np.linalg.matrix_power((R * AAR_inv), i)

    if correctbias_w:
        HRM_k = HRM_k + 1/(1-w) * np.linalg.matrix_power((R * AAR_inv), (k+1))

    # aproximation of (A^T A)^{-1}
    return AAR_inv * HRM_k

def main():
    Amn = np.mat([[1,0], [0,0.1]]) # numpy.matrix (a little bit slow)
    dist_bn = np.mat([[1], [1]])

    vas, ves = np.linalg.eigh(Amn.T * Amn)

    Amn_dim = Amn.shape
    Rnn = np.zeros((Amn_dim[1], Amn_dim[1]))
    for i in range(Amn_dim[1]-1, Amn_dim[1], 1) :
        Rnn[i][i] = min((vas[0] ** 2 + vas[0] * vas[Amn_dim[1]-1] )** 0.5, vas[1])

    k = 1
    w = 0.0
    correct_bias_w = True
    # x_hr = hrm(Amn, Rnn, k, w) * Amn.T * dist_bn
    x_hr = hrm(Amn, Rnn, k, w, correct_bias_w) * Amn.T * dist_bn

    print(x_hr)

if __name__ == '__main__':

    try:
        main()

    except:
        pass
