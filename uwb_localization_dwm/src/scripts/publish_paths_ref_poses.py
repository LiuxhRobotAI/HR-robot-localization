#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from queue import Queue
# import threading
# mutex_lock = threading.Lock() # RLock

poseQueue = Queue(maxsize=0)

def publish_pose(pose_pub):
    if poseQueue.qsize():
        pose_pub.publish(poseQueue[-1])


def path_callback(pa_data, agrs):
# def path_callback(pa_data):

    # global pose_pub
    pose_pub = agrs[0]

    if len(pa_data.poses) :
        pose_pub.publish(pa_data.poses[-1])
        # mutex_lock.acquire(blocking=True)
        poseQueue.put(pa_data.poses[-1])
        # mutex_lock.release()


def main():
    """
    Initial node
    """

    try:
        # Initial a path
        rospy.init_node('PathToPose')

        path_topic = ['/aft_mapped_path', '/vins_estimator/path', '/loop_fusion/pose_graph_path']

        # global pose_pub
        pose_pub = rospy.Publisher(path_topic[0]+'/pose', PoseStamped, queue_size=50)

        pose_pub1 = rospy.Publisher(path_topic[1]+'/pose', PoseStamped, queue_size=50)

        # rospy.Subscriber(path_topic[0], Path, path_callback)
        dict_2 = 0
        rospy.Subscriber(path_topic[0], Path, path_callback,(pose_pub, dict_2))

        rospy.Subscriber(path_topic[1], Path, path_callback,(pose_pub1, dict_2))

        # initial rate
        rate = rospy.Rate(50)

        # Tests if it is running, then updates data
        while not rospy.is_shutdown():
            # publish_pose(pose_pub)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except :
        print("Error: Unkown error, please check.")
        pass

if __name__ == '__main__':
    # pose_pub = None

    main()
