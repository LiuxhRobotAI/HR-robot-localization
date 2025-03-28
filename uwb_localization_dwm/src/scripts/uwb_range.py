"""
A class of a UWB anchor range measurements to a specific tag and the anchor positions.
"""

import string
from numpy import *
import rospy
from geometry_msgs.msg  import Point
from uwb_localization_dwm.msg import UWBrange

class uwb_range:

    def __init__(self, anchor_id: string, to_tag_id: string, anchor_position: Point,
                 distance:float64, flag: int = None) :
        self.anchor = UWBrange()
        self.anchor.header.stamp = rospy.Time.now()
        self.anchor.anchor_id = anchor_id
        self.anchor.to_tag_id = to_tag_id
        self.anchor.anchor_position = anchor_position
        self.anchor.distance_to_tag = distance

        try:
            self.flag = flag
        except NameError:
            self.flag = None
        except:
            pass
