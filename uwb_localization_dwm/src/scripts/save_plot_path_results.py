#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

'''
(1)
Save path in ROS bag topics in txt as TUM format: timestamp(s) tx ty tz qx qy qz qw,
and plot the paths.
Anther way to save TUM format is evo tool with the following command:
    evo_traj bag name.bag /topic_name  --save_as_tum
(2)
Analyse the results.

(3)
(ROS base) env
'''

import rosbag
import sys
import os

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d # for 3D plot
import numpy as np
from scipy import stats

if len(sys.argv) < 2 :
    bagname = '2024-10-13-17-13-33_8'

else:
    bagname =sys.argv[1]
bagpath = '/media/liu/SSD2T/jackal/rosbag_data/'
basepath = '/home/liu/workspace/ws/data/jackal_bag/rosbag_restore_xh/'

bagfile = bagpath + bagname+'.bag'

topics_list=['/uwb/localization/tag/hr_path',
             '/uwb/localization/tag/ls_path',
             '/aft_mapped_path',
             '/vins_estimator/path',
             '/loop_fusion/pose_graph_path',
             ]

try:
    bag = rosbag.Bag(bagfile, "r")
except:
    print("Error: Invalid bag path or bag name, please check.\n")
    print(bagfile)
    sys.exit()

# Save path
result_paths =[]
for i in range(len(topics_list)):
    bag_data = bag.read_messages(topics=topics_list[i]) # topic, msg, t(s)

    if None == bag_data:
        print("No required data in bag") # Not working
        continue

    result_paths.append(basepath+'uwb_path_results/' + bagname+'_'+ os.path.basename(topics_list[i])+'.txt')
    if None ==result_paths:
        break

    with open(result_paths[i], "w", encoding='utf-8') as file:
        for topic, msg, t in bag_data:
        # for topic, msg, t in bag.read_messages(): # Or, call all topics in the bag
            if msg is not None:
                # TUM format
                file.write(str(msg.header.stamp.to_sec()) +' '+ str(msg.poses[-1].pose.position.x) + ' ' + \
                        str(msg.poses[-1].pose.position.y) + ' ' + str(msg.poses[-1].pose.position.z) + ' ' + \
                        str(msg.poses[-1].pose.orientation.x) +' '+ str(msg.poses[-1].pose.orientation.y) +' '+ \
                        str(msg.poses[-1].pose.orientation.z)+' '+ str(msg.poses[-1].pose.orientation.w) + '\n')

bag.close()

# Plot
anchors_coordinate = []
uwb_data = {}
path_poses=[]
for i in range(len(result_paths)):
    path_poses.append([])

for i in range(len(result_paths)):
    with open(result_paths[i], "r", encoding='utf-8') as file :
        path_poses[i] = [j[:-1].split(' ') for j in file.readlines()]

for i in range(len(result_paths)):
    # pos = path_poses[i][:][1:4] # [1,4): x,y,z; not array()
    pos = [j[1:4] for j in path_poses[i]] # posistions in string list

    try: # some paths are not record
        posA = np.zeros((len(pos),len(pos[0])), dtype = float)
        for k in range(len(pos[0])):
            posA[:,k]=list(map(np.float64, [i[k] for i in pos]))
        posA = np.array(posA)

        plt.plot(posA)
        plt.legend(['x', 'y', 'z'], loc='upper right')
        plt.title('Trajectory of the robot')

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        # ax.plot3D(list(map(float, [i[0] for i in pos])), list(map(float, [i[1] for i in pos])), list(map(float, [i[2] for i in pos])),'gray')
        ax.plot3D(posA[:,0], posA[:,1], posA[:,2],'gray')
        ax.set_title('3D line plot')
        plt.savefig(basepath+'uwb_path_results/' + bagname+'_'+ os.path.basename(topics_list[i])+'.png')
        plt.show()
    except:
        print("Error: Invalid path: " + topics_list[i] + ", please check.\n")
        continue
