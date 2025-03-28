#!/usr/bin/env python3

import matplotlib.pyplot as plt
import random
import numpy as np
import sys
from scipy import stats, signal
from scipy.fft import fft, fftfreq


if len(sys.argv) < 2 :
    filename = '2023-11-29-16-54-18_098_0.0_0.0_0.3'

else:
    filename =sys.argv[1]
filepath = '/home/liu/workspace/ws/data/jackal_bag/rosbag_restore_xh/uwb_dis/'
txtfile = filepath + filename +'.txt'

with open(txtfile, "r", encoding='utf-8') as file :
    anchor_data = [i[:-1].split(' ') for i in file.readlines()]

uwb_dis_all =[]
num_ref = 20
ref_dis = np.zeros(num_ref)
for i in range(len(anchor_data)):
    uwb_dis_all.append(float(anchor_data[i][-1])) # For different format data.

uwb_dis = uwb_dis_all[0:-1]

dis_mean = np.mean(uwb_dis)
dis_std = np.std(uwb_dis)

dis_bias = uwb_dis - dis_mean
bias_mean = np.mean(dis_bias)
bias_std = np.std(dis_bias)
bias_mean_arr = bias_mean * np.ones_like(dis_bias)
bias_abs_mean = np.mean(np.abs(dis_bias)) # Only for showwing the abs mean
bias_abs_std = np.std(np.abs(dis_bias))

uwb_dis_tf = np.cbrt(uwb_dis) # sqrt,log,cbrt

dis_skew = stats.skew(uwb_dis)
dis_kurtosis = stats.kurtosis(uwb_dis)
dis_skew_tf = stats.skew(uwb_dis_tf)
dis_kurtosis_tf = stats.kurtosis(uwb_dis_tf)

print('Skewness: '+ str(dis_skew), ', Kurtosis: '+ str(dis_kurtosis))
print('Skewness : '+ str(dis_skew_tf), ', Kurtosis: '+ str(dis_kurtosis_tf))

plt.figure()
plt.subplot(211)
plt.plot(uwb_dis,'b')
plt.ylabel('Value')
plt.xlabel('Point index')
plt.title('Measurement')
plt.subplot(212)
plt.plot(dis_bias,'p')
plt.ylabel('Value')
plt.xlabel('Point index')
plt.title('Bias to mean')

plt.figure()
plt.subplot(311)
plt.hist(uwb_dis, bins=int(len(uwb_dis)/10), edgecolor='black')
plt.xlabel('Value')
plt.ylabel('Frequency')
plt.title('Histogram')
hist, bn_edges = np.histogram(uwb_dis)
cdf = stats.cumfreq(uwb_dis)
plt.plot(bn_edges[1:], cdf[0])

plt.subplot(312)
plt.hist(uwb_dis_tf, bins=int(len(uwb_dis)/10), edgecolor='black')
plt.xlabel('Transformated value')
plt.ylabel('Frequency')
plt.title('Histogram')

plt.subplot(313)
plt.text(0,int(len(dis_bias)/10), 'Mean(Abs): '+ str(format(bias_abs_mean,'.4f')) +
            ', Std(Abs): '+ str(format(bias_abs_std,'.4f')) + ', \n'+
            'Mean: '+ str(format(bias_mean,'.4f')) +
            ', Std: ' + str(format(bias_std,'.4f')) +
            ', Skewness: '+ str(format(dis_skew,'.4f')) +
            ', Kurtosis: '+ str(format(dis_kurtosis,'.4f')), va='center', ha='center')
# plt.hist(stats.norm.pdf(dis_bias), bins='auto', density=True, alpha=0.6, edgecolor='black')
plt.hist(dis_bias, bins=int(len(dis_bias)/10), edgecolor='black')
plt.xlabel('Bias to mean')
plt.ylabel('Frequency')
plt.title('Histogram')
plt.tight_layout()
# plt.show()


## Spectrum analysis
# frequecy
fs = 1e2
N = len(dis_bias)

# Fourier transform
yf = fft(dis_bias)
xf = fftfreq(N, 1/fs)[:N//2]

plt.figure()
plt.subplot(211)
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
plt.xlabel('Frequency (B:Hz)');   plt.ylabel('Amplitude (A)')
plt.grid()
# plt.show()

# spectrum analysis
F, T, Sxx = signal.spectrogram(dis_bias, fs=fs) # freqs, time, Spectrogram of x
plt.subplot(212)
plt.pcolormesh(T, F, Sxx, shading='auto') # shading='flat'; auto=nearest, gouraud
plt.ylabel('Frequency (B: Hz)')
plt.xlabel('Time (s)')
# plt.show()

fs = 1e2 # sampling frequency; different from sensor frequency 10Hz
# Fs_max = 5 # maximum frequency of the signal (if only consider the movement, then less than 0.1)
fc = 3 # cutoff frequency
wn = 2*fc/fs # normalized cutoff frequency = fc/nyquist_freq; nyquist_freq = 0.5 fs
filter_order = 5 # 8
b, a = signal.butter(filter_order, wn, 'lowpass')  # filter configuration
filted_uwb_dis = signal.filtfilt(b, a, uwb_dis)
plt.figure()
plt.subplot(211)
plt.plot(filted_uwb_dis,'b')
plt.ylabel('Value')
plt.xlabel('Point index')
plt.title('Filtered measurement')
plt.subplot(212)
plt.hist(filted_uwb_dis, bins=int(len(filted_uwb_dis)/10), edgecolor='black')
plt.xlabel('Value')
plt.ylabel('Frequency')
plt.title('Histogram')
plt.tight_layout()
plt.show()
