#!/usr/bin/env python3

import rospy, time, serial, os, sys, random

from std_msgs.msg       import Float64
from geometry_msgs.msg  import Pose, PoseStamped, Vector3Stamped, Point
from nav_msgs.msg import Path
import tf

# from geometry_msgs.msg  import Pose
# from geometry_msgs.msg  import PoseStamped
# from std_msgs.msg       import Float64

from numpy import *
import numpy as np
from typing import Tuple


from dwm1001c_apiCommands import DWM1001_API_COMMANDS

# new class to store uwb message
from uwb_range import uwb_range
from uwb_localization_dwm.msg import UWBrange
from filtered_distance import filtered_distance, filter_uwb_dis
from uwb_particle_filter import uwb_particle_filter, pfilter_dis

from hrm import hrm
from lsm import lsm

from uwb_ring import ring_buffer

from queue import LifoQueue
import threading

class UwbData:
    def __init__(self) :
        self.distance_queue = LifoQueue(maxsize=0)
        self.d_queue_tmp = LifoQueue(maxsize=0) # for rosbag distances
        self.mutex_lock = threading.Lock()
        self.q_anchor_set = set() # the anchors in distance_queue

uwb_bag_data = UwbData()

class UwbLocalizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """

        # Init node
        rospy.init_node('uwb_localozation_{}'.format(random.randint(0,100000)), anonymous=False)

        # Get port and tag name
        self.dwm_port = rospy.get_param('~port')
        self.tag_name = rospy.get_param('~tag_name')
        self.use_network = rospy.get_param('~use_network', False)
        self.network = rospy.get_param('~network', "DTPAUWB")
        self.verbose = rospy.get_param('~verbose', False)

        self.problem_dimension = rospy.get_param('~problem_dimension', 3)
        self.min_anchors = rospy.get_param('~min_anchors', 4)
        self.is_from_bag = rospy.get_param('~uwb_from_bag')

        # Set a ROS rate
        # self.rate = rospy.Rate(1)
        self.rate = rospy.Rate(10) # Set a higher publish rate.

        # Empty dictionary to store topics being published
        self.topics = {}

        # Empty dictionary to store paths being published
        self.path_record = {}

        self.fud = filter_uwb_dis(filterN=100,fs=2e2,fc=3,filter_order=5) # lowpass filter

        self.pfd = pfilter_dis()

        if self.is_from_bag:
            self.node_ids = ['AN0' ,'AN1','AN2','AN3','AN4']
            for i in range(len(self.node_ids)):
                tag_dis_topic = '/dwm1001' + "{}".format("/"+self.network if self.use_network else "") + \
                                '/tag/' + self.tag_name + '/to/anchor/' + self.node_ids[i] + "/distance"
                rospy.Subscriber(tag_dis_topic, UWBrange, tag_dis_callback)

        else:
            # Serial port settings
            self.serialPortDWM1001 = serial.Serial(
                port = self.dwm_port,
                baudrate = 115200,
                parity = serial.PARITY_ODD,
                stopbits = serial.STOPBITS_TWO,
                bytesize = serial.SEVENBITS
            )

    def main(self) :
        """
        Initialize data measurenment api
        :param:
        :returns: none
        """

    # def create_path(self) :
        # create and initial a path to record the pose
        lsm_path_record = "ls_path_record"
        if lsm_path_record not in self.path_record :
            self.path_record[lsm_path_record] = Path()

        hrm_path_record = "hr_path_record"
        if hrm_path_record not in self.path_record :
            self.path_record[hrm_path_record] = Path()

        if self.is_from_bag:
            first_time = True
            rospy.loginfo("Waiting for uwb data ...")

            try:
                while not rospy.is_shutdown():
                    if (len(uwb_bag_data.q_anchor_set) < self.min_anchors):
                        continue

                    try :
                        Amn, bm = self.data_loader()
                    except:
                        rospy.loginfo("Fail to load correct data, waiting for new uwb data ...")
                        continue

                    if 0==np.all(bm):
                        rospy.loginfo("States are unobservable, waiting for new uwb data ...")
                        continue

                    try:
                        self.localization(Amn, bm)
                        if first_time:
                            first_time = False
                            rospy.loginfo("The localizer is publishing the path ...")
                    except np.linalg.LinAlgError as err:
                        if 'Singular matrix' in str(err):
                            rospy.loginfo("Please check the singular matrix, unable to calculate the position.")
                        pass

            except KeyboardInterrupt:
                rospy.loginfo("Quitting localization ...")

            return

        """
        Initialize port and dwm1001 api
        """

        self.serialPortDWM1001.close()

        time.sleep(1)

        self.serialPortDWM1001.open()


        if(self.serialPortDWM1001.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001.name) )

            self.initializeDWM1001API()

            time.sleep(2)

            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001 coordinates")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

        try:
            while not rospy.is_shutdown():

                serialReadLine = self.serialPortDWM1001.read_until()

                try:
                    self.publishTagPositions(serialReadLine)

                except IndexError:
                    rospy.loginfo("Found index error in the network array! DO SOMETHING!")

        except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")

            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.rate.sleep()
            serialReadLine = self.serialPortDWM1001.read_until()
            if b"reset" in serialReadLine:
                rospy.loginfo("succesfully closed ")
                self.serialPortDWM1001.close()

    def data_loader(self) -> Tuple[array,array]:
        if (uwb_bag_data.distance_queue.qsize() < self.min_anchors):
            rospy.logwarn('Size of distance_queue : {}, skiping.'.format(uwb_bag_data.distance_queue.qsize()))
            raise Exception('Not enough uwb data.')

        dim_anchor_position = 3 # sensor measurement dimension

        anum = self.min_anchors # at least 4 anchors are required

        An_positions = np.zeros([anum, dim_anchor_position], dtype=float)
        to_tag_dist = np.zeros(anum, dtype=float)
        anchors = set() # {}

        uwb_bag_data.mutex_lock.acquire(blocking=True)

        for i in range(anum) :
            dis_q = uwb_bag_data.distance_queue.get()
            An_positions[i][0] = dis_q.anchor_position.x
            An_positions[i][1] = dis_q.anchor_position.y
            An_positions[i][2] = dis_q.anchor_position.z

            anchor_coordinate = str(An_positions[i][0])+'_'+ str(An_positions[i][1])+'_'+ str(An_positions[i][2])
            if anchor_coordinate not in self.fud.anchors_coordinate :
                self.fud.anchors_coordinate[anchor_coordinate]=ring_buffer(maxlen=self.fud.filterN)

            self.fud.anchors_coordinate[anchor_coordinate].append(dis_q.distance_to_tag)

            if anchor_coordinate not in self.pfd.anchors_coordinate :
                self.pfd.anchors_coordinate[anchor_coordinate]= \
                    uwb_particle_filter(particleN=300, x0=dis_q.distance_to_tag, P0=0.1, Q=1, R=0.03, dt=0.1,
                        option={'methods':None, 'v_max':0.2,'alpha':2,'lamda':50,'cte':0.03,})

            if len(self.fud.anchors_coordinate[anchor_coordinate].data) == self.fud.filterN:
                to_tag_dist[i] = filtered_distance(self.fud.fs, self.fud.fc, self.fud.filter_order,
                                    self.fud.anchors_coordinate[anchor_coordinate].get())[-1]
            else:
                to_tag_dist[i] = dis_q.distance_to_tag
                to_tag_dist[i] ,_,_ =self.pfd.anchors_coordinate[anchor_coordinate].update_estimate(float64(to_tag_dist[i]))
                self.pfd.anchors_coordinate[anchor_coordinate].resample()

            anchors.add(dis_q.anchor_id)
        uwb_bag_data.distance_queue.queue.clear()
        uwb_bag_data.q_anchor_set.clear()
        uwb_bag_data.mutex_lock.release()

        if not anchors.issuperset(self.node_ids[:anum]):
            # rospy.logerr("Not enough data.")
            rospy.loginfo('Found same ids, number of found anchor id : {}.'.format(len(anchors)))

        anchors.clear()

        m = anum - 1
        dim_n = int(self.problem_dimension)
        Amn = np.zeros([m, dim_n], dtype=float)
        bm = np.zeros([m, 1], dtype=float)

        for i in range(m) :
            Amn[[i], :] = np.array([(An_positions[i][0]-An_positions[m][0]),
                                    (An_positions[i][1]-An_positions[m][1]),
                                    (An_positions[i][2]-An_positions[m][2])]) # use anchors' data
            bm[i][0] = 0.5*((An_positions[i][0]**2-An_positions[m][0]**2+ \
                             An_positions[i][1]**2-An_positions[m][1]**2+ \
                             An_positions[i][2]**2-An_positions[m][2]**2+ \
                             to_tag_dist[m]**2-to_tag_dist[i]**2))

        return Amn, bm

    def pathUpdating(self, path_pub, path_record:Path, add_pose:"PoseStamped" = None) :
        """
        Data update: Path update, tf update
        """

        # Time stamp
        current_time = rospy.Time.now()

        # Publish tf
        br = tf.TransformBroadcaster()
        br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                         current_time, "uwb", "map")

        # configure the path, adding the current pose to the path
        path_record.header.stamp = current_time
        path_record.header.frame_id = 'uwb'
        path_record.poses.append(add_pose)

        # Publish the path
        path_pub.publish(path_record)
        if self.verbose:
            print("Published the path.") # ToDo

    # Publish topics with disatance and position informaion seperately, which causes a problme for subscription.
    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray: Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """

        arrayData = [x.strip() for x in serialData.strip().split(b',')]

        # The number of elements should be 2 + 6*NUMBER_OF_ANCHORS + 5 (TAG POS)
        data_head_length = 2
        anchor_data_length = 6
        tag_data_length = 5

        # If getting a tag position
        if b"DIST" in arrayData[0] :

            number_of_anchors = 0
            if b"POS" in arrayData[-tag_data_length] :
                number_of_anchors = int((len(arrayData) - data_head_length - tag_data_length) / anchor_data_length)
            else :
                number_of_anchors = int((len(arrayData) - data_head_length) / anchor_data_length)

            # Get the anchors positions, distances and tag's position
            for i in range(number_of_anchors) :
                node_id = str(arrayData[2+6*i], 'UTF8')
                first_time = False
                if node_id not in self.topics :
                    first_time = True
                    # The publish rate and the time synchronic should be considered
                    self.topics[node_id] = rospy.Publisher(
                        '/dwm1001' +
                        "{}".format("/"+self.network if self.use_network else "") +
                        '/anchor/' + node_id +
                        "/position",
                        PoseStamped,
                        queue_size=100
                    )
                    self.topics[node_id+"_dist"] = rospy.Publisher(
                        '/dwm1001' +
                        "{}".format("/"+self.network if self.use_network else "") +
                        '/tag/' + self.tag_name +
                        '/to/anchor/' + node_id +
                        "/distance",
                        UWBrange,
                        queue_size=100
                    )
                try :
                    # transform the data from port to geometry_msgs/PoseStamped
                    p = PoseStamped()
                    p.header.stamp = rospy.Time.now()
                    p.pose.position.x = float(arrayData[data_head_length+2+6*i])
                    p.pose.position.y = float(arrayData[data_head_length+3+6*i])
                    p.pose.position.z = float(arrayData[data_head_length+4+6*i])
                    p.pose.orientation.x = 0.0
                    p.pose.orientation.y = 0.0
                    p.pose.orientation.z = 0.0
                    p.pose.orientation.w = 1.0
                    self.topics[node_id].publish(p)
                except :
                    pass
                try :
                    # anchor_name = str(arrayData[data_head_length+6*i+1], 'UTF8')
                    to_tag_id = self.tag_name
                    dist = float(arrayData[data_head_length+5+6*i])
                    An_dis = uwb_range(node_id, to_tag_id, p.pose.position, dist, 0)
                    An_dis.anchor.header.stamp = p.header.stamp
                    self.topics[node_id+"_dist"].publish(An_dis.anchor)
                except :
                    pass

                if self.verbose or first_time :
                    rospy.loginfo("Anchor " + node_id + ": "
                                  + " x: " + str(p.pose.position.x)
                                  + " y: " + str(p.pose.position.y)
                                  + " z: " + str(p.pose.position.z))

            # Now publish the position of the tag itself
            if b"POS" in arrayData[-5] :

                # Topic is now a tag with same name as node_id
                if self.tag_name not in self.topics :
                    self.topics[self.tag_name] = \
                        rospy.Publisher('/dwm1001/tag/'+self.tag_name+"/position",
                                        PoseStamped, queue_size=100)

                # transform the data from port to geometry_msgs/PoseStamped
                p = PoseStamped()
                p.header.stamp = rospy.Time.now()
                p.pose.position.x = float(arrayData[-4])
                p.pose.position.y = float(arrayData[-3])
                p.pose.position.z = float(arrayData[-2])
                p.pose.orientation.x = 0.0
                p.pose.orientation.y = 0.0
                p.pose.orientation.z = 0.0
                p.pose.orientation.w = 1.0
                self.topics[self.tag_name].publish(p)

                # Show the tag position itself
                if self.verbose or first_time :
                    rospy.loginfo("Tag " + self.tag_name + ": "
                                  + " x: " + str(p.pose.position.x)
                                  + " y: " + str(p.pose.position.y)
                                  + " z: " + str(p.pose.position.z))

        # Publish the calculated tag's positions
        if b"DIST" in arrayData[0] and 0 != int(arrayData[1]) :
            min_anchors = self.min_anchors
            if int(arrayData[1]) < min_anchors :
                rospy.loginfo("Warn: not enough anchors to locate a tag, less than %d.", min_anchors)

            else :
                if self.verbose or first_time :
                    rospy.loginfo("Calculating tag's position ...")

                data_anchor_num = int(arrayData[1])

                anum = 0
                if b"POS" in arrayData[-tag_data_length] :
                    anum = int((len(arrayData) - data_head_length - tag_data_length) /
                            anchor_data_length)
                else :
                    anum = int((len(arrayData) - data_head_length) / anchor_data_length)

                try :
                    self.problem_dimension
                except NameError:
                    rospy.logwarn("No problem_dimension defined!")
                    self.problem_dimension = 3 # 3 dimension problems
                except :
                    pass
                dim_n = int(self.problem_dimension)
                dim_anchor_position = 3 # sensor measurement dimension = 3

                if anum == data_anchor_num :
                    An_positions = np.zeros([anum, dim_anchor_position], dtype=float)
                    to_tag_dist = np.zeros(anum, dtype=float)

                    for i in range(anum) :
                        try :
                           # restore all coordinates
                           An_positions[i][0] = float(arrayData[data_head_length+2+6*i])
                           An_positions[i][1] = float(arrayData[data_head_length+3+6*i])
                           An_positions[i][2] = float(arrayData[data_head_length+4+6*i])
                        except :
                           pass
                        try :

                            anchor_coordinate = str(An_positions[i][0])+'_'+ str(An_positions[i][1])+'_'+ str(An_positions[i][2])
                            if anchor_coordinate not in self.fud.anchors_coordinate : # Add a new anchor and data buffer
                                self.fud.anchors_coordinate[anchor_coordinate]=ring_buffer(maxlen=self.fud.filterN)


                            self.fud.anchors_coordinate[anchor_coordinate].append(float(arrayData[data_head_length+5+6*i]))

                            if anchor_coordinate not in self.pfd.anchors_coordinate :
                                self.pfd.anchors_coordinate[anchor_coordinate]= \
                                    uwb_particle_filter(particleN=300, x0=float(arrayData[data_head_length+5+6*i]), P0=0.1, Q=1,
                                        R=0.03, dt=0.1, option={'methods':None, 'v_max':0.2,'alpha':2,'lamda':50,'cte':0.03,})

                            if len(self.fud.anchors_coordinate[anchor_coordinate].data) == self.fud.filterN:
                                to_tag_dist[i] = filtered_distance(self.fud.fs, self.fud.fc, self.fud.filter_order,
                                                    self.fud.anchors_coordinate[anchor_coordinate].get())[-1]
                            else:
                                to_tag_dist[i] = float(arrayData[data_head_length+5+6*i])
                                to_tag_dist[i] ,_,_ =self.pfd.anchors_coordinate[anchor_coordinate].update_estimate(float64(to_tag_dist[i]))
                                self.pfd.anchors_coordinate[anchor_coordinate].resample()

                        except :
                            pass

                    # Build mathematical model
                    m = anum - 1
                    Amn = np.zeros([m, dim_n], dtype=float)
                    bm = np.zeros([m, 1], dtype=float) # dim_m
                    for i in range(m) :
                        # use anchors' data
                        Amn[[i], :] = np.array([(An_positions[i][0]-An_positions[m][0]),
                                                (An_positions[i][1]-An_positions[m][1]),
                                                (An_positions[i][2]-An_positions[m][2])])
                        bm[i][0] = 0.5*((An_positions[i][0]**2-An_positions[m][0]**2+ \
                                         An_positions[i][1]**2-An_positions[m][1]**2+ \
                                         An_positions[i][2]**2-An_positions[m][2]**2+ \
                                         to_tag_dist[m]**2-to_tag_dist[i]**2))

                    try:
                        self.localization(Amn, bm)
                        if first_time:
                            rospy.loginfo("The localizer is publishing the path ...")
                    except np.linalg.LinAlgError as err:
                        if 'Singular matrix' in str(err):
                            rospy.loginfo("Please check the singular matrix, unable to calculate the position.")
                        pass

    def localization(self, Amn: array, bm: array):
        # Calculate the position

        x_ls = lsm(Amn, bm)
        lsm_pose_topic = "lsm_localziation"
        if lsm_pose_topic not in self.topics :
            self.topics[lsm_pose_topic] = \
                rospy.Publisher('/uwb/localization/'+self.tag_name+"/ls_position", PoseStamped, queue_size=100)

        ls_p = PoseStamped()
        ls_p.header.stamp = rospy.Time.now()
        ls_p.header.frame_id = 'uwb'
        ls_p.pose.position.x = float(x_ls[0])
        ls_p.pose.position.y = float(x_ls[1])
        if 3 == int(self.problem_dimension) :
            ls_p.pose.position.z = float(x_ls[2])
        else :
            ls_p.pose.position.z = 0.0

        ls_p.pose.orientation.x = 0.0
        ls_p.pose.orientation.y = 0.0
        ls_p.pose.orientation.z = 0.0
        ls_p.pose.orientation.w = 1.0
        self.topics[lsm_pose_topic].publish(ls_p)

        # Show the tag position itself
        if self.verbose:
            rospy.loginfo("Tag " + self.tag_name + " ls_position : "
                            + " x: " + str(ls_p.pose.position.x)
                            + " y: " + str(ls_p.pose.position.y)
                            + " z: " + str(ls_p.pose.position.z))

        # Path update for lsm
        lsm_path_topic = "lsm_path_pub"
        if lsm_path_topic not in self.topics :
            self.topics[lsm_path_topic] = rospy.Publisher('/uwb/localization/'+self.tag_name+"/ls_path",
                                                          Path, queue_size=100)

        try :
            self.pathUpdating(self.topics[lsm_path_topic], self.path_record["ls_path_record"], ls_p)
            if self.verbose:
                print("The {} published the path ...\n".format(lsm_path_topic), end="\n")
        except :
            rospy.loginfo("Unable to update path with the pose!")
            pass

        # # high order regularization method
        Amn = np.mat(Amn) # numpy.matrix (a little bit slow)
        bm = np.mat(bm)
        vas, ves = np.linalg.eigh(Amn.T * Amn)

        # calculate the regularization matrix
        Amn_dim = Amn.shape
        Rnn = np.zeros((Amn_dim[1], Amn_dim[1]))
        for i in range(Amn_dim[1]-1, Amn_dim[1], 1) :
            Rnn[i][i] = min(abs(vas[0]**2 + vas[0]* vas[Amn_dim[1]-1])**0.5, vas[Amn_dim[1]-2])

        e, v = np.linalg.eig(Amn.T * Amn)
        min1, min2, max1 = np.sort(e)[[0, 1, -1]]
        min1_index = np.argsort(e)[0]
        Rnn_e = np.zeros(Amn_dim[1])
        Rnn_e[min1_index] = min(abs(min1**2 + min1* max1)**0.5, min2)
        Rnn[min1_index, min1_index] = min(abs(min1**2 + min1* max1)**0.5, min2)

        k = 1
        w = 0.0
        correct_bias_w = True
        x_hr = hrm(Amn, Rnn, k, w, correct_bias_w) * Amn.T * bm

        hrm_node = "hrm_localziation"
        if hrm_node not in self.topics :
            self.topics[hrm_node] = rospy.Publisher('/uwb/localization/'+self.tag_name+"/hr_position",
                                                    PoseStamped, queue_size=100)

        hr_p = PoseStamped()
        hr_p.header.stamp = rospy.Time.now()
        hr_p.header.frame_id = 'uwb'
        hr_p.pose.position.x = float(x_hr[0])
        hr_p.pose.position.y = float(x_hr[1])
        if 3 == int(self.problem_dimension) :
            hr_p.pose.position.z = float(x_hr[2])

        else :
            hr_p.pose.position.z = 0.0

        hr_p.pose.orientation.x = 0.0
        hr_p.pose.orientation.y = 0.0
        hr_p.pose.orientation.z = 0.0
        hr_p.pose.orientation.w = 1.0
        self.topics[hrm_node].publish(hr_p)

        if self.verbose:
            rospy.loginfo("Tag " + self.tag_name + " hr_position : "
                            + " x: " + str(hr_p.pose.position.x)
                            + " y: " + str(hr_p.pose.position.y)
                            + " z: " + str(hr_p.pose.position.z))

        # Path update for hrm
        hrm_path_topic = "hrm_path_pub"
        if hrm_path_topic not in self.topics :
            self.topics[hrm_path_topic] = rospy.Publisher('/uwb/localization/'+self.tag_name+"/hr_path",
                                                          Path, queue_size=100)

        try :
            self.pathUpdating(self.topics[hrm_path_topic], self.path_record["hr_path_record"], hr_p)
            if self.verbose:
                print("The {} published the path ...\n".format(hrm_path_topic), end="\n")
        except :
            rospy.loginfo("Unable to update path with the pose!")
            pass

    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes
        :param:
        :returns: none
        """

        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)

        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        time.sleep(0.5)

        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

# # subscription tag-anchor pair distance
def tag_dis_callback(uwb_data):
    uwb_bag_data.mutex_lock.acquire(blocking=True)

    if 0 == uwb_bag_data.distance_queue.qsize() and not uwb_bag_data.d_queue_tmp.empty():
        while not uwb_bag_data.d_queue_tmp.empty():
            tmp_q = uwb_bag_data.d_queue_tmp.get()
            anchor_coordinate = str(tmp_q.anchor_position.x) +'_'+ \
                str(tmp_q.anchor_position.y)+'_'+ str(tmp_q.anchor_position.z)

            if anchor_coordinate in uwb_bag_data.q_anchor_set: # Don't want repeat anchors
                continue

            uwb_bag_data.q_anchor_set.add(anchor_coordinate)
            uwb_bag_data.distance_queue.put(tmp_q)

    anchor_coordinate = str(uwb_data.anchor_position.x)+'_'+ \
        str(uwb_data.anchor_position.y)+'_'+ str(uwb_data.anchor_position.z)

    if anchor_coordinate not in uwb_bag_data.q_anchor_set:
        uwb_bag_data.q_anchor_set.add(anchor_coordinate)
        uwb_bag_data.distance_queue.put(uwb_data)

    else:
        uwb_bag_data.d_queue_tmp.put(uwb_data)

    uwb_bag_data.mutex_lock.release()

if __name__ == '__main__':
    # global initial pose for a path
    x, y, z, tr, tp, th = 0, 0, 0, 0, 0, 0

    try:
        localizer = UwbLocalizer() # localizer with the published distances and anchors' postions
        localizer.main()

    except rospy.ROSInterruptException:
        pass
