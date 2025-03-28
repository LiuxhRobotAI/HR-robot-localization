#!/usr/bin/env python3

'''
Fuhry's Tihkonov regularization method (Fuhry, 2012) with optimal parameter.
'''

from numpy import *
import numpy as np
from spectral_radius import spectral_radius

def ftr(Amn: array, bm: array, method:str=None, miu_in:float=None) -> array:
    A = np.mat(Amn)
    bm=np.reshape(bm,(3,1))
    b = np.mat(bm)

    # A = USV'=sum_i^l(s_i*u_i*v_i'); A_mpi=sum_i^l(1/s_i*v_i*u_i')
    [U,s,V] = np.linalg.svd(A, full_matrices=1,compute_uv=1)
    # print(s)
    S = np.diag(s)
    A_k_inv = np.zeros((S.T).shape)
    kn = np.linalg.matrix_rank(A)
    X_TSVD = np.zeros(((S.T).shape[0],1))

    if kn <= len(A):
        miu2 = np.sqrt(2*(S[0,0]**2)/(S[kn-1,kn-1]**2)) # svd(A)**2 = eig(A.T*A)
        if miu2>= S[kn-1,kn-1]**2 :
            miu2 = min(miu2, S[kn-2,kn-2]**2)
        elif miu2 <= S[kn-2,kn-2]**2:
            miu2 = max(miu2, S[kn-1,kn-1]**2)
    else:
        miu2 = S[kn-1,kn-1]

    if miu_in:
        miu2=miu_in

    # Truncated SVD
    D_m2 = np.zeros(S.shape)
    for i in range(0,kn-1) :
        s_i = S[i,i]
        u_i = U[:,i]
        v_i = V[:,i]
        D_m2[i,i] = max(miu2-s_i**2, 0)
        A_k_inv_i = 1/s_i*v_i*u_i.T
        A_k_inv = A_k_inv + A_k_inv_i
        x_Ti = 1/s_i*v_i*u_i.T * b
        X_TSVD = X_TSVD + x_Ti

    # new Tikhonov method (FTR; Fuhry, 2012)
    X_fTr = X_TSVD
    for i in range(kn-1, kn):
        s_i = S[i,i]
        u_i = U[:,i]
        v_i = V[:,i]
        D_m2[i,i] = max(miu2-S[i,i]**2, 0)
        if S[i-1,i-1]**2>=miu2 and miu2>=S[i,i]**2:
            D_m2[i,i] = miu2-S[i,i]**2
        A_k_inv_i = 1/s_i*v_i*u_i.T
        A_k_inv = A_k_inv + A_k_inv_i
        x_Ti = (s_i**2/miu2)*1/s_i*v_i*u_i.T * b
        X_fTr = X_fTr + x_Ti # [Yang, 2015] and [Fuhry, 2012]
    # X_fTr = V*np.mat((S.T*S + D_m2)).I*S.T*U.T* b # [Fuhry, 2012]

    if method == 'TSVD':
        return X_TSVD

    return X_fTr

def main():
    Amn = np.mat([[1,0,0], [0,2,0], [0,0,3]]) # 3D
    dist_bn = np.mat([[1], [1], [1]])

    vas, ves = np.linalg.eigh(Amn.T * Amn)

    Amn_dim = Amn.shape
    Rnn = np.zeros((Amn_dim[1], Amn_dim[1]))
    for i in range(Amn_dim[1]-1, Amn_dim[1], 1) :
        Rnn[i][i] = (vas[0] ** 2 + vas[0] * vas[Amn_dim[1]-1] ) ** 0.5
        # Rnn[i][i] = (sigma_ATA[Amn_dim[1]-1] ** 2 + sigma_ATA[Amn_dim[1]-1] * sigma_ATA[1] ) ** 0.5

    x_fTr = ftr(Amn, dist_bn)

    print(x_fTr)

if __name__ == '__main__':

    main()