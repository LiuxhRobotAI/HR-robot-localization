#!/usr/bin/env python3

import matplotlib.pyplot as plt
import sys
from scipy import signal
from numpy import float64
import numpy as np

from collections import deque
from uwb_ring import ring_buffer

class filter_uwb_dis:
    """ class that implements a not-yet-full buffer """
    def __init__(self,filterN,fs,fc,filter_order):
        self.filterN = filterN
        self.fs = fs
        self.fc = fc
        self.filter_order = filter_order
        self.anchors_coordinate = {} # ring_buffer(maxlen=filterN) of each anchor
        # self.filtered_N_dis = np.zeros(self.filterN)

def filtered_distance(fs=1e2, fc=3, filter_order:int= 5, uwb_dis:float64=None,option=None, x0=None)-> float:
    if None is uwb_dis:
        return None

    wn = 2*fc/fs # normalized cutoff frequency
    b, a = signal.butter(filter_order, wn, 'lowpass')
    if None==option :
        filtered_dis = signal.filtfilt(b, a, uwb_dis)

    if 'online'==option:
        b, a = signal.butter(3, 0.05, 'lowpass')
        if x0:
            zi = signal.lfilter_zi(b, a)
            filtered_dis, zf = signal.lfilter(b, a, [uwb_dis], zi=zi*x0) # online
        else:
            filtered_dis = signal.lfilter(b, a, [uwb_dis])

    return filtered_dis

def main():
    if len(sys.argv) < 2 :
        filename = '2024-10-13-17-13-33_8_7.2_0.5_3.0'
    else:
        filename =sys.argv[1]
    filepath = '/home/liu/workspace/ws/data/jackal_bag/rosbag_restore_xh/uwb_dis/'
    txtfile = filepath + filename +'.txt'

    with open(txtfile, "r", encoding='utf-8') as file :
        anchor_data = [i[:-1].split(' ') for i in file.readlines()]

    uwb_dis_all =[]

    for i in range(len(anchor_data)):
        uwb_dis_all.append(float(anchor_data[i][3]))

    uwb_dis = uwb_dis_all[0:]

    fs=2e2
    fc=3
    filter_order= 5

    filtered_uwb_dis =[]
    filterN = 100

    fud = filter_uwb_dis(filterN,fs,fc,filter_order)

    for i in range(len(uwb_dis)):
        anchor_coordinate = str(uwb_dis[0])+'_'+ str(uwb_dis[1])+'_'+ str(uwb_dis[2])
        if anchor_coordinate not in fud.anchors_coordinate : # Add a new anchor and data buffer
            fud.anchors_coordinate[anchor_coordinate]=ring_buffer(maxlen=filterN)

        fud.anchors_coordinate[anchor_coordinate].append(uwb_dis[i])

        if len(fud.anchors_coordinate[anchor_coordinate].data) == filterN:
            filtered_uwb_dis.append(
                filtered_distance(fud.fs, fud.fc, fud.filter_order,
                                  fud.anchors_coordinate[anchor_coordinate].get())[-1])
        else:
            filtered_uwb_dis.append(uwb_dis[i])

    option=None # ='online'
    if option:
        filtered_uwb_dis =np.zeros_like(uwb_dis)
        for i in range(len(uwb_dis)):
            filtered_uwb_dis[i] = filtered_distance(fs, fc, filter_order, uwb_dis[i],option)

    plt.figure()
    plt.subplot(311)
    plt.plot(filtered_uwb_dis,'b')
    plt.ylabel('Value')
    plt.xlabel('Point index')
    plt.title('Filtered measurement')
    plt.subplot(312)
    plt.hist(filtered_uwb_dis, bins=int(len(filtered_uwb_dis)/10), edgecolor='black')
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.title('Histogram')

    plt.subplot(313)
    plt.plot(uwb_dis,'b')
    plt.ylabel('Value')
    plt.xlabel('Point index')
    plt.title('Measurement')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
