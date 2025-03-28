#!/usr/bin/env python3

import rospy
from numpy import *
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
import tf
import math
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt
from typing import Tuple

from tf.transformations import quaternion_from_matrix, euler_from_matrix

from queue import Queue, LifoQueue
import threading

num_Rt_poses = 400
uwbQueue = LifoQueue(maxsize=0)
odomQueue = LifoQueue(maxsize=0)
odomQueue_cache = LifoQueue(maxsize=0)
mutex_lock = threading.Lock() # RLock
# jackal_odom_path=None,
# jo_path_pub=None

mutex_lock1 = None
synchronization = None # mean nothing if assign as None, equal last line

def rigid_transform3D(A, B):
    """
    Input: expects Nx3 matrix of points
    Returns R,t
    R = 3x3 rotation matrix
    t = 3x1 column vector
    Note:
    Should include 's' to minimize the error duo to two sensor configure in the body (Umeyama alignment)
    or consider the body configure directory.
    """

    assert len(A) == len(B)

    N = A.shape[0]  # total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    H = np.matmul(np.transpose(AA),BB)
    U, __, Vt = np.linalg.svd(H) # U, S, Vt
    R = np.matmul(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = np.matmul(Vt.T,U.T)

    # s = VarA / np.trace(np.diag(D) @ S) if with_scale else 1.0 # , with_scale: bool = False # ToDo
    t = -np.matmul(R, centroid_A) + centroid_B
    # err = B - np.matmul(A,R.T) - t.reshape([1, 3])
    return R, t

def uwb_callback(u_data):
    # global odomQueue, uwbQueue
    if len(u_data.poses) :
        mutex_lock.acquire(blocking=True)
        uwbQueue.put(u_data.poses[-1].pose)
        mutex_lock.release()
        # global synchronization
        if True == synchronization :
            if None == odomQueue_cache :
                mutex_lock.acquire(blocking=True)
                uwbQueue.get() # wait for odom
                mutex_lock.release()
                print("Warn: No pose in odomQueue_cache!")
                return

        # if odomQueue_cache :
            # global mutex_lock1
            # mutex_lock1.acquire(blocking=True)
            odomq = odomQueue_cache.get() # unable to lock with get() and if, and may cause dead lock
            # mutex_lock1.release()

            mutex_lock.acquire(blocking=True)
            odomQueue.put(odomq) # synchronization error
            odomQueue_cache.queue.clear()
            mutex_lock.release()

def odom_callback(o_data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', len(o_data.poses))
    # if !len(o_data.poses):
    # return

    if len(o_data.poses) :
        global synchronization

        if True == synchronization :
            try :
                if o_data :
                    mutex_lock.acquire(blocking=True)
                    odomQueue_cache.put(o_data.poses[-1].pose)
                    mutex_lock.release()
            except:
                    print("No msg in odom queue cache.")
        else :
            mutex_lock.acquire(blocking=True)
            odomQueue.put(o_data.poses[-1].pose)
            mutex_lock.release()

def uwb_odom_trojectory(path_pub, u_path_pub, o_pose_pub=None, u_pose_pub=None, odom_path:"Path"=None, uwb_path:"Path"=None, simulation=None):
    """
    Transformation update: calibaration two frames with two paths in a slide window: Path update, tf update
    """

    # Time stamp
    current_time = rospy.Time.now()

    if None == odom_path :
        print("Warn: Adding a path for odometry.") # ToDo
        odom_path = Path()

    if None == uwb_path :
        print("Warn: Adding a path for uwb.")
        uwb_path = Path()

    # configure pose of the Path
    if None == simulation :
        simulation = True

        # create simulation movement
        global x, y, z, tr, tp, th

        dt = 1 / 50
        vx = 0.25
        vy = 0.25
        vz = 0
        vth = 0.2
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_z = 0*vz
        delta_z = 0
        delta_th = vth * dt
        x += delta_x
        y += delta_y
        z += delta_z
        tr = 0
        tr = 0
        th += delta_th

        # quaternion transfermation
        quat = tf.transformations.quaternion_from_euler(0, 0, th)
        # print(tf.transformations.quaternion_from_euler(0,0,0)) # q = (0,0,0,1)

        o_pose = PoseStamped()
        o_pose.header.stamp = current_time
        o_pose.header.frame_id = 'odom'
        o_pose.pose.position.x = x
        o_pose.pose.position.y = y
        o_pose.pose.position.z = z
        o_pose.pose.orientation.x = quat[0]
        o_pose.pose.orientation.y = quat[1]
        o_pose.pose.orientation.z = quat[2]
        o_pose.pose.orientation.w = quat[3]
        o_pose_pub.publish(o_pose)

        u_pose = PoseStamped()
        # u_pose = o_pose # only copy a reference not create a new class, if assigned for a new value then the reference change
        u_pose.header.frame_id = 'uwb'
        Txyz_u_o = [1.0, 2.0, 0.5]
        R_u_o = [0.0, 0.0, 0.0, 1.0]
        u_pose.pose.position.x = x + Txyz_u_o[0] # Suppose the transformation is not all zero (RT.x=1)
        u_pose.pose.position.y = y + Txyz_u_o[1]
        u_pose.pose.position.z = z + Txyz_u_o[2]
        u_pose.pose.orientation.x = quat[0] + R_u_o[0]
        u_pose.pose.orientation.y = quat[1] + R_u_o[1]
        u_pose.pose.orientation.z = quat[2] + R_u_o[2]
        u_pose.pose.orientation.w = quat[3] + R_u_o[3]
        u_pose_pub.publish(u_pose)

        # Publish tf (transform between two frames)
        br = tf.TransformBroadcaster()
        # t = geometry_msgs.msg.TransformStamped()
        # br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(), "odom", "map") # 'odom' -> 'map'; 'map' == 'world'
        t = TransformStamped()
        t.header.frame_id = "map"
        t.header.stamp = current_time
        # t.child_frame_id = "odom"
        t.child_frame_id = o_pose.header.frame_id
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1.0
        # transform from tf frame 'odom' -> 'map'
        br.sendTransform((t.transform.translation.x,
                         t.transform.translation.y,
                         t.transform.translation.z),
                         (t.transform.rotation.x,
                         t.transform.rotation.y,
                         t.transform.rotation.z,
                         t.transform.rotation.w),
                         t.header.stamp,
                         t.child_frame_id,
                         t.header.frame_id)

        Txyz_u_m = Txyz_u_o
        R_u_m = R_u_o
        br.sendTransform(Txyz_u_m, R_u_m, rospy.Time.now(),
                         u_pose.header.frame_id, "map") # 'odom' -> 'map'; 'map' == 'world'

    # configure the path, adding the current pose to the path
    odom_path.header.stamp = current_time
    odom_path.header.frame_id = o_pose.header.frame_id
    odom_path.poses.append(o_pose)

    uwb_path.header.stamp = current_time
    uwb_path.header.frame_id = u_pose.header.frame_id
    uwb_path.poses.append(u_pose)

    # limit poses in the path # No necessary, only for simulation path
    if simulation :
        if len(odom_path.poses) > 1000:
            odom_path.poses.pop(0)

        if len(uwb_path.poses) > 1000:
            uwb_path.poses.pop(0)

    # Publish the paths
    path_pub.publish(odom_path)
    u_path_pub.publish(uwb_path)

def jackal_odom_callback(msg): #:"PoseStamped"
    # global jackal_odom_path, jo_path_pub
    # print(msg)
    # jackal_odom_path.header.stamp = msg.header.stamp
    # jackal_odom_path.header.frame_id = msg.header.frame_id
    # jackal_odom_path.poses.append(msg.pose)
    # jo_path_pub.publish(jackal_odom_path)
    try :
        global synchronization
        if synchronization :
            mutex_lock1.acquire(blocking=True)
            odomQueue_cache.put(msg.pose.pose)
            mutex_lock1.release()
    except:
            print("No msg in jackal odom.")

def calculateRT(odomQueue, uwbQueue) -> Tuple[ndarray, ndarray] :# #from typing import Tuple
    """
    Calibaration two frames with two paths in a slide window.
    Input:
        Two pose queues of the same trojectory that described in in two diffrence frames, respectively.
    Ouptut:
    R = 3x3 rotation matrix
    t = 3x1 column vector
    """

    if odomQueue.qsize() != uwbQueue.qsize():
        odomQueue.queue.clear()
        uwbQueue.queue.clear()
        print("Error: the Queues size is not equal!", end="\n")
        return

    p_o = np.zeros([num_Rt_poses, 3])
    p_u = np.zeros([num_Rt_poses, 3])
    # pose_o = PoseStamped()
    # pose_u = PoseStamped()

    mutex_lock.acquire(blocking=True)
    try :
        for i in range(num_Rt_poses) :
            pose_o = odomQueue.get()
            p_o[i, 0] = pose_o.position.x
            p_o[i, 1] = pose_o.position.y
            p_o[i, 2] = pose_o.position.z

            pose_u = uwbQueue.get()
            p_u[i, 0] = pose_u.position.x
            p_u[i, 1] = pose_u.position.y
            p_u[i, 2] = pose_u.position.z

    except IndexError:
        print("Error: index error in the Queue(s)!", end="\n")
    except :
        print("Error: undefined situations, please check!", end="\n")
        pass
    finally:
        # print(odomQueue.qsize(), uwbQueue.qsize())
        odomQueue.queue.clear()
        uwbQueue.queue.clear()
    mutex_lock.release()

    # TODO: should calculate the difference of two queues to determine if (R,t) is reliable
    R,t = rigid_transform3D(p_o, p_u)

    return R, t

def updateRt(num_Rt_poses :int):
    # if uwbQueue.qsize() >= num_Rt_poses and uwbQueue.qsize() == odomQueue.qsize() :
    if uwbQueue.qsize() >= num_Rt_poses :
        if uwbQueue.qsize() == odomQueue.qsize() :
            try :
                R, t = calculateRT(odomQueue, uwbQueue)
                print('R:\n',R , end="\n")
                print('t:\n', t, end="\n")
                al, be, ga = euler_from_matrix(R, 'rxyz')
                # Euler_angle = print(type(Euler))
                Euler_angle = np.array([al, be, ga])*180/np.pi
                print('Euler_angle: \n', Euler_angle, end="\n") #al, be, ga = euler_from_matrix(Re, 'rxyz')

                # R = tf.transformations.rotation_matrix(0.123, (1, 2, 3))
                T = np.identity(4)
                T[:3, :3] = R
                # q = quaternion_from_matrix(T)
                br = tf.TransformBroadcaster()
                # transform from tf frame 'odom' -> 'map'
                br.sendTransform(t, quaternion_from_matrix(T), rospy.Time.now(), 'odom', 'uwb')
                # odomQueue.queue[0].header.frame_id, uwbQueue.queue[0].header.frame_id

            except:
                print("Warn: can't calculate the rotation and translation of the two given frames,", end="\n")
                pass

def calibration(simulation=None):
    """
    Initial node
    """

    global synchronization, mutex_lock1, num_Rt_poses
    synchronization = False # software synchronization, or False
        # Need a new lock for a new queue.
    if True == synchronization :
        mutex_lock1 = threading.Lock()

    if None == simulation:
        simulation = False

    print('Calibration simulation = ', simulation)

    try:
        # Initial a path
        rospy.init_node('PathRecord')

        if True == simulation :
            # creat a publisher path_pub to publish a path with the topic 'trajectory'
            o_path_pub = rospy.Publisher('odom/trajectory', Path, queue_size=50)
            u_path_pub = rospy.Publisher('uwb/trajectory', Path, queue_size=50)

            o_pose_pub = rospy.Publisher('odom/pose', PoseStamped, queue_size=50)
            u_pose_pub = rospy.Publisher('uwb/pose', PoseStamped, queue_size=50)

            u_path_sub = rospy.Subscriber("/uwb/trajectory", Path, uwb_callback)
            rospy.Subscriber('/odom/trajectory', Path, odom_callback) # nav_msgs/Path
        else:
            synchronization = True # software synchronization, or False
            mutex_lock1 = threading.Lock()
            # pubulish jackal path
            # global jackal_odom_path, jo_path_pub
            # jackal_odom_path = Path()
            # jo_path_pub = rospy.Publisher('jackal_velocity_controller/odom/path', Path, queue_size=50)
            # rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, jackal_odom_callback)

            # odom0_path_topic = rospy.get_param('~odom0_path_topic') #
            odom0_path_topic = "/uwb/localization/tag/hr_path"
            # odom1_path_topic = rospy.get_param('~odom1_path_topic') # 
            odom1_path_topic = '/aft_mapped_path'

            # calibration with real jackal and uwb
            rospy.Subscriber(odom0_path_topic, Path, uwb_callback)
            # rospy.Subscriber('/jackal_velocity_controller/odom/path', Path, odom_callback)
            # rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, jackal_odom_callback) # nav_msgs/Odometry
            rospy.Subscriber(odom1_path_topic, Path, odom_callback)


        # initial rate
        rate = rospy.Rate(50)

        # create and initial a path to record the pose
        path_record = Path()
        u_path_record = Path()

        # Tests if it is running, then updates data
        while not rospy.is_shutdown():
            # Update poses and the path
            # uwb_odom_calibration(o_path_pub, u_path_pub, path_record, u_path_record)
            if simulation :
                uwb_odom_trojectory(o_path_pub, u_path_pub, o_pose_pub, u_pose_pub, path_record, u_path_record)

            updateRt(num_Rt_poses)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except :
        print("Error: Unkown error, please check.")
        pass

if __name__ == '__main__':
    # # Initial state of movement
    x, y, z, tr, tp, th = 0, 0, 0, 0, 0, 0
    simulation = False # True

    calibration(simulation)
