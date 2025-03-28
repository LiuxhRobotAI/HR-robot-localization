#!/usr/bin/env python3

"""
A new Tihkionov method provide by Fuhry Fuhry,2012 and use the proposed regularization parameter.
Save the trajectory as TUM format in a txt file with the same timestamp from a anchor.
The distances are read from the dataset of each ROS bag results.

"""

import sys
from numpy import *
import numpy as np
import pandas as pd

from filtered_distance import filtered_distance, filter_uwb_dis
from uwb_particle_filter import uwb_particle_filter, pfilter_dis
from uwb_ring import ring_buffer

from lsm import lsm
from hrm import hrm
from ftr import ftr

if len(sys.argv) < 2 :
    dinfo = ['2024-10-13-17-13-33_8']
else:
    dinfo =[sys.argv[1], sys.argv[2]]
bagpath = '/home/liu/workspace/ws/data/jackal_bag/rosbag_restore_xh/'

# # New lab, narrow in y
ancs= ['1.5_2.0_0.69',
       '1.5_0.17_2.0',
       '7.17_2.0_1.5',
       '7.2_0.5_3.0']

dfile_name= ['uwb_dis/'+dinfo[0] +'_' +ancs[0] +'.txt',
            'uwb_dis/'+dinfo[0] +'_' +ancs[1] +'.txt',
            'uwb_dis/'+dinfo[0] +'_' +ancs[2] +'.txt',
            'uwb_dis/'+dinfo[0] +'_' +ancs[3] + '.txt']

path_name=bagpath+'uwb_path_results/'+dinfo[0]+'_saved_dis_path_' # +'.txt'

def save_uwb_path(p, f_name):
    with open(f_name, "w", encoding='utf-8') as file:
        for i in range(len(p)):
            # TUM format
            file.write(str(p[i][0]) +' '+ str(p[i][1]) +' '+str(p[i][2]) +' '+ str(p[i][3]) + ' ' + \
                       '0.0' + ' ' +'0.0' + ' ' +'0.0' + ' ' + '1.0' + '\n')

def getdata() -> [ndarray, ndarray, ndarray]:
    """
    Get and store all UWB anchor positions and range measurements to a specific tag.
    """

    # load all distanes in a trajectory
    an_data=[]
    use_data_cols=[0,3,4,5,6]
    for i in range(len(dfile_name)):
        an_data.append(pd.read_table(bagpath + dfile_name[i], sep=' ',header=None,usecols=use_data_cols))
        # an_data.append(pd.read_table(bagpath + bagname[i], sep=' ',header=None,usecols=[0,3,4,5,6]))
    uwb_data_size = []

    # find the minimum data of the same trajectory
    for i in range(len(ancs)):
        uwb_data_size.append(len(an_data[i][6]))
    k = min(uwb_data_size)

    k_t = an_data[0][0][0:k] # use the timestamp of this anchor
    m = len(ancs)-1
    dim_n = 3
    Amn = np.zeros([m, dim_n], dtype=float)
    bm = np.zeros([m, 1], dtype=float)
    # bm = np.zeros(m, dtype=float)

    k_Amn = np.zeros([k, m, dim_n], dtype=float)
    k_bm = np.zeros([k, m, 1], dtype=float)
    An_positions = np.zeros([len(ancs), dim_n], dtype=float)
    to_tag_dist = np.zeros(len(ancs), dtype=float)

    fud = filter_uwb_dis(filterN=100,fs=2e2,fc=3,filter_order=5) # lowpass filter

    pfd = pfilter_dis()

    for t in range(k): # different time instant
        for i in range(len(ancs)): # different anchors
            An_positions[i] = an_data[i].loc[t,[3,4,5]]
            anchor_coordinate = str(An_positions[i][0])+'_'+ str(An_positions[i][1])+'_'+ str(An_positions[i][2])

            if anchor_coordinate not in fud.anchors_coordinate : # Add a new anchor and data buffer
                fud.anchors_coordinate[anchor_coordinate]=ring_buffer(maxlen=fud.filterN)

            if anchor_coordinate not in pfd.anchors_coordinate : # Create a pf for a new anchor data
                pfd.anchors_coordinate[anchor_coordinate]= \
                    uwb_particle_filter(particleN=300, x0=float(an_data[i][6][0]), P0=0.1, Q=1,
                        R=0.03, dt=0.1, option={'methods':None, 'v_max':0.2,'alpha':2,'lamda':50,'cte':0.03,})

            if len(fud.anchors_coordinate[anchor_coordinate].data) == fud.filterN:
                to_tag_dist[i] = filtered_distance(fud.fs, fud.fc, fud.filter_order,
                                    fud.anchors_coordinate[anchor_coordinate].get())[-1]
            else:
                to_tag_dist[i] = an_data[i][6][t]
                to_tag_dist[i] ,_,_ =pfd.anchors_coordinate[anchor_coordinate].update_estimate(float64(to_tag_dist[i]))
                pfd.anchors_coordinate[anchor_coordinate].resample()

        for i in range(m) :
            Amn[[i], :] = np.array([(An_positions[i][0]-An_positions[m][0]),
                                    (An_positions[i][1]-An_positions[m][1]),
                                    (An_positions[i][2]-An_positions[m][2])]) # use anchors' data
            bm[i] = 0.5*((An_positions[i][0]**2-An_positions[m][0]**2+ \
                                An_positions[i][1]**2-An_positions[m][1]**2+ \
                                An_positions[i][2]**2-An_positions[m][2]**2+ \
                                to_tag_dist[m]**2-to_tag_dist[i]**2))

        k_Amn[t] = Amn
        k_bm[t] = bm

    return k_t, k_Amn, k_bm


def FTR_localizer():
    [ts, k_Amn, k_bn] = getdata()
    x_ls = np.zeros([len(k_bn),3])
    x_hr = np.zeros([len(k_bn),3])
    x_oftr = np.zeros([len(k_bn),3]) # Fuhry's Tihkonov method(FTR) with optimal R
    x_tsvd = np.zeros([len(k_bn),3]) # TSVD (FTR or HR with smallest miu2)
    x_ftr_m = np.zeros([len(k_bn),3]) # Fuhry's Tihkonov method with largest miu2
    x_ftr_m2 = np.zeros([len(k_bn),3]) # Fuhry's Tihkonov method with a setting miu2

    for t in range(len(k_bn)):
        Amn=k_Amn[t]
        bm=np.array(np.reshape(k_bn[t], (3,)))

        # LS
        x_ls[t] = np.reshape(lsm(Amn, bm),(3,))

        # HRM
        Amn = np.mat(Amn) # numpy.matrix (a little bit slow)
        bm = np.mat(bm)

        vas, P = np.linalg.eigh(Amn.T * Amn)

        # calculate the regularization matrix
        Amn_dim = Amn.shape
        Rnn = np.zeros((Amn_dim[1], Amn_dim[1]))
        for i in range(Amn_dim[1]-1, Amn_dim[1], 1) :
            # Rnn[i][i] = abs(vas[0]**2 + vas[0]* vas[Amn_dim[1]-1])**0.5 # asb(): for unexpected errors.
            Rnn[i][i] = min(abs(vas[0]**2 + vas[0]* vas[Amn_dim[1]-1])**0.5, vas[Amn_dim[1]-2])
        # The transfor should only change the position of nonzero item.
        # eigh()--> lamda(i): small to large. So no need P here, or set Rnn[1][1]=miu first.
        # Rnn = P*Rnn*P.T
        # print(Rnn[i][i],vas)

        # Find the smallest eigenvalue, regardless of which component
        e, v = np.linalg.eig(Amn.T * Amn)
        min1, min2, max1 = np.sort(e)[[0, 1, -1]]
        min1_index = np.argsort(e)[0]
        Rnn_e = np.zeros(Amn_dim[1])
        Rnn_e[min1_index] = min(abs(min1**2 + min1* max1)**0.5, min2)
        # Rnn[min1_index][min1_index] = min(abs(min1**2 + min1* max1)**0.5, min2)
        Rnn = np.diag(Rnn_e)
        # Rnn = v * np.diag(Rnn_e) * np.linalg.inv(v) # tested
        Rnn = np.array(Rnn)

        k = 1
        lam, _ = np.linalg.eig(Rnn*(Amn.T* Amn + Rnn).I); lam_min = np.min(lam) # 0,0.96
        w = 0.0
        correct_bias_w = True
        bm=np.reshape(bm, (3,1))
        x_hr[t] = np.reshape(hrm(Amn, Rnn, k, w, correct_bias_w) * Amn.T * bm, (3,))

        # Optimal Fuhry Tihkonov Regularization
        x_oftr[t] =np.reshape(ftr(Amn, bm),(3,))
        # With the optimal miu2 => (HR,k=0)
        # Equivalent to HR,k=0
        k = 0
        # w = 0.96
        correct_bias_w=False
        miu2 = np.sqrt(2*vas[Amn_dim[1]-1]/vas[0])
        if miu2>= vas[Amn_dim[1]-1] :
            miu2 = min(miu2, vas[Amn_dim[1]-2])
            Rnn[Amn_dim[1]-1][Amn_dim[1]-1]=miu2
        elif miu2 <= vas[Amn_dim[1]-2]:
            miu2 = max(miu2, vas[Amn_dim[1]-1])
            Rnn[Amn_dim[1]-1][Amn_dim[1]-1]=miu2
        # x_oftr[t] = np.reshape(hrm(Amn, Rnn, k, w, correct_bias_w) * Amn.T * bm, (3,)) # ！= ftr()?

        x_tsvd[t] = np.reshape(ftr(Amn, bm, method='TSVD'), (3,))

        miu_l = vas[Amn_dim[1]-1] # set a largest miu2 with eig(n-1)
        # miu_in = (vas[Amn_dim[1]-2,Amn_dim[1]-2]) # equil to TSVD
        x_ftr_m[t] = np.reshape(ftr(Amn, bm, miu_in=miu_l), (3,))

        # miu_in2 = vas[Amn_dim[1]-1]/2 + vas[Amn_dim[1]-2]/2 # similar performance
        miu_in2 = sqrt(vas[Amn_dim[1]-1]*vas[0]) # use sqrt(largest*smallest svd) of A (eig Amn.T * Amn)
        x_ftr_m2[t] = np.reshape(ftr(Amn, bm, miu_in=miu_in2), (3,))

    # p = [ts, x_ls]
    ts = np.reshape(np.array(ts),(len(ts),1))
    p_tls = np.append(ts, x_ls, axis=1)
    f_name = path_name+'ls'+'.txt'
    save_uwb_path(p_tls, f_name)

    p_thr = np.append(ts, x_hr, axis=1)
    f_name = path_name+'hr'+'.txt'
    save_uwb_path(p_thr, f_name)

    p_toftr = np.append(ts, x_oftr, axis=1)
    f_name = path_name+'ftr'+'.txt'
    save_uwb_path(p_toftr, f_name)

    p_ttsvd = np.append(ts, x_tsvd, axis=1)
    f_name = path_name+'tsvd'+'.txt'
    save_uwb_path(p_ttsvd, f_name)

    p_tftr_m = np.append(ts, x_ftr_m, axis=1)
    f_name = path_name+'ftr_m'+'.txt'
    save_uwb_path(p_tftr_m, f_name)

    p_tftr_m2 = np.append(ts, x_ftr_m2, axis=1)
    f_name = path_name+'ftr_m2'+'.txt'
    save_uwb_path(p_tftr_m2, f_name)

if __name__ == '__main__':
    FTR_localizer()
