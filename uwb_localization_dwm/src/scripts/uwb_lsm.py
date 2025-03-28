#!/usr/bin/env python3

"""
A simulation to test the LSM.
"""

import rospy
from numpy import *
import numpy as np

from geometry_msgs.msg  import Point, PoseStamped

from lsm import lsm
from uwb_range import uwb_range as uwb_range
from uwb_localization_dwm.msg import UWBrange

def getdata() -> uwb_range:
# """
# Get and store all UWB anchor positions and range measurements to a specific tag.
# """

    anchor_position = Point()
    anchor_position.x = 0.1
    anchor_position.y = 0.2
    anchor_position.z = 0.3
    An0 = uwb_range('0', '1', anchor_position, 3, 0)

    return An0


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)


def lsm_localizer():
    rospy.init_node('uwb_lsm', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        An_dis = getdata()
        rospy.loginfo("Anchor_id: " + str(An_dis.anchor.anchor_id) + ", position:"
                       + " x: " + str(An_dis.anchor.anchor_position.x)
                       + " y: " + str(An_dis.anchor.anchor_position.y)
                       + " z: " + str(An_dis.anchor.anchor_position.z)
                       + ", to_tag_distance: " + str(An_dis.anchor.distance_to_tag)
                       + ", to_tag_id: " + str(An_dis.anchor.to_tag_id)
                      )

        pub_dis = rospy.Publisher(
                            '/uwb/localization' + '/anchor' + "/to_tag_distance",
                            UWBrange,
                            queue_size=100
                            )
        pub_dis.publish(An_dis.anchor)

        Amn = np.mat([[1,0], [0,2]])
        bn = np.mat([[1], [1]])

        x_ls = lsm(Amn, bn)

        pub = rospy.Publisher(
                            '/uwb/localization' + '/tag' + "/ls_position",
                            PoseStamped,
                            queue_size=100
                            )
        try :
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.pose.position.x = float(x_ls[0])
            p.pose.position.y = float(x_ls[1])
            p.pose.position.z = 0
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            pub.publish(p)
        except :
            pass

        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try :
        lsm_localizer()

    except rospy.ROSInterruptException :
        pass
