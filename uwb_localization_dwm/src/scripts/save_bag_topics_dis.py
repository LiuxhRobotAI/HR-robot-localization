#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

'''
Save anchor distances in bag topics in txt as:
    timestamp(s) x y z dis.
And split distances into seperate file correspoding to each anchors using their coordinates.
Almost the same as:
rostopic echo -b bag_name.bag -p /topic_name > file_name.txt # | [file_name.cvs]
'''

import rosbag
import sys
import os

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

if len(sys.argv) < 2 :
    bagname = '2024-10-13-17-13-33_8'
else:
    bagname =sys.argv[1]
bagpath = '/media/liu/SSD2T/jackal/rosbag_data/'
basepath = '/home/liu/workspace/ws/data/jackal_bag/rosbag_restore_xh/'

bagfile = bagpath + bagname+'.bag'
anchors_list= ['AN0','AN1','AN2','AN3']
topics_list=['/dwm1001/tag/tag/to/anchor/'+anchors_list[0]+'/distance',
             '/dwm1001/tag/tag/to/anchor/'+anchors_list[1]+'/distance',
             '/dwm1001/tag/tag/to/anchor/'+anchors_list[2]+'/distance',
             '/dwm1001/tag/tag/to/anchor/'+anchors_list[3]+'/distance',] # '/dwm1001/tag/tag/to/anchor/AN0/distance'

try:
    bag = rosbag.Bag(bagfile, "r")
except:
    print("Error: Invalid bag path or bag name, please check.\n")
    print(bagfile)
    sys.exit()
bag_info = bag.get_type_and_topic_info() # rosbag info bag_name
# print(bag_info)

bag_data = bag.read_messages(topics=topics_list) # topic, msg, t(s)

file = open(basepath+'uwb_dis/' + bagname+'_'+ 'AN_dis.txt', "w", encoding='utf-8')
for topic, msg, t in bag_data:
# for topic, msg, t in bag.read_messages(): # Or, call all topics in the bag
    if msg is not None:
        # print(msg.anchor_id)

        file.write(str(msg.header.stamp.to_sec()) +' '+ str(msg.anchor_id)+' ' +str(msg.to_tag_id)+' '+ \
                   str(msg.anchor_position.x) + ' ' + str(msg.anchor_position.y) + ' ' + \
                   str(msg.anchor_position.z) + ' ' +  str(msg.distance_to_tag) +'\n')
file.close()
bag.close()

anchors_coordinate = []
uwb_data = {}
an_dis_groups = {}
Ans_file = basepath+ 'uwb_dis/' + bagname+'_'+ 'AN_dis.txt'
with open(Ans_file, "r", encoding='utf-8') as file :
    an_dis = [i[:-1].split(' ') for i in file.readlines()]

for i in range(len(an_dis)):
    anchor_coordinate = an_dis[i][3:6] # [3,6)
    anchor_coordinate_str=anchor_coordinate[0]+'_' +anchor_coordinate[1]+'_' +anchor_coordinate[2]
    if anchor_coordinate not in anchors_coordinate :
        anchors_coordinate.append(anchor_coordinate)

    if anchor_coordinate_str not in uwb_data :
            uwb_data[anchor_coordinate_str] = []
            an_dis_groups[anchor_coordinate_str] = []

    if anchor_coordinate_str in uwb_data:
        uwb_data[anchor_coordinate_str].append([float(anchor_coordinate[0]),
                                                float(anchor_coordinate[1]),
                                                float(anchor_coordinate[2]),
                                                float(an_dis[i][6])])
        an_dis_groups[anchor_coordinate_str].append(an_dis[i])

# write distance table in groups; alternative pandas.groupby([3,4,5])
for i in range(len(an_dis_groups)): # key: anchor distance group
    dis_file = basepath+ 'uwb_dis/' + bagname+'_'+ list(an_dis_groups.keys())[i] + '.txt'
    with open(dis_file, "w", encoding='utf-8') as file :
        # file.writelines(an_dis_groups[list(an_dis_groups.keys())[i]])
        for line in range(len(an_dis_groups[list(an_dis_groups.keys())[i]])): # list line index
            for idx, s in enumerate(an_dis_groups[list(an_dis_groups.keys())[i]][line]):
                if idx==len(an_dis[i])-1:
                    file.write(s)
                else:
                    file.write(s+' ')
            file.write('\n')

# uwb_data_arrs =[]
for key in uwb_data:
    # print(x[0] for x in uwb_data[anchor_coordinate_str])
    uwb_data_arr = np.array(uwb_data[key])
    # uwb_data_arrs=np.insert(uwb_data_arrs, uwb_data_arr)

    dis_skew = stats.skew(uwb_data_arr[:,3]) # Skewness(posibility of bias, tail direction)：Nomal distribution =0
    dis_kurtosis = stats.kurtosis(uwb_data_arr[:,3]) # Kurtosis(samples near to mean, peak): Nomal =0 (3)
    print('Skewness: '+ str(dis_skew), ', Kurtosis: '+ str(dis_kurtosis))

    dis_mean = np.mean(uwb_data_arr[:,3])
    dis_std = np.std(uwb_data_arr[:,3])
    dis_mean_arr = dis_mean * np.ones_like(uwb_data_arr[:,3])
    plt.figure()
    plt.subplot(411)
    plt.text(0,dis_mean+0.1, 'Mean: '+ str(format(dis_mean,'.4f'))+', Std: '+ str(format(dis_std,'.4f'))
             +', Skewness: '+ str(format(dis_skew,'.4f')) +', Kurtosis: '
             + str(format(dis_kurtosis,'.4f')) , va='top', ha='left')
    plt.plot(uwb_data_arr[:,3],'b', dis_mean_arr,'k')
    plt.title('UWB data processing')
    plt.ylabel('Distance')
    plt.xlabel('Point index')
    plt.legend(['AN_'+key, 'Mean'], loc='upper right')

    dis_bias = uwb_data_arr[:,3] - dis_mean
    bias_abs_mean = np.mean(np.abs(dis_bias)) # Only for showwing the abs mean
    bias_abs_std = np.std(np.abs(dis_bias))

    bias_mean = np.mean(dis_bias)
    bias_std = np.std(dis_bias)
    bias_mean_arr = bias_mean * np.ones_like(dis_bias)

    plt.subplot(412)
    plt.text(0,bias_abs_mean+0.05, 'Mean(Abs): '+ str(format(bias_abs_mean,'.4f'))+
             ', Std(Abs): '+ str(format(bias_abs_std,'.4f')) +
             ', Mean: '+ str(format(bias_mean,'.4f')) +
             ', Std: ' + str(format(bias_std,'.4f')), va='top', ha='left')
    plt.plot(dis_bias,'p')
    plt.title('UWB bias to mean')
    plt.ylabel('Distance')
    plt.xlabel('Point index')
    plt.legend(['Bias'], loc='upper right')

    plt.subplot(413)
    plt.hist(uwb_data_arr[:,3], bins=int(len(uwb_data_arr)/10), edgecolor='black')
    plt.xlabel('Measurement')
    plt.ylabel('Frequency')
    plt.title('Histogram and CDF')
    hist, bn_edges = np.histogram(uwb_data_arr[:,3])
    cdf = stats.cumfreq(uwb_data_arr[:,3])
    plt.plot(bn_edges[1:], cdf[0])

    plt.subplot(414)
    plt.hist(dis_bias, bins=int(len(dis_bias)/10), edgecolor='black')
    plt.xlabel('Bias to mean')
    plt.ylabel('Frequency')
    plt.title('Histogram')
    # plt.hist(stats.norm.pdf(dis_bias), bins='auto', density=True, alpha=0.2, edgecolor='black')

    plt.tight_layout()
    plt.savefig(basepath +'uwb_dis/'+ bagname +'_'+ key +'.png')
    # plt.close()

plt.show()
