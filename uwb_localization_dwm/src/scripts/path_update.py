#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
import tf
import math

# # Initial state of movement
# # x, y, th = 0, 0, 0
# x, y, z, tr, tp, th = 0, 0, 0, 0, 0, 0

def pathUpdating(path_pub, path_record:Path, pose:"PoseStamped"=None):
    """
    Data update: Path update, tf update
    """

    # Time stamp
    current_time = rospy.Time.now()

    # configure pose of the Path
    if None == pose :
        simulation = True
        # create simulation movement
        global x, y, z, tr, tp, th

        dt = 1 / 50
        vx = 0.25
        vy = 0.25
        # vz = 0
        vth = 0.2
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        # delta_z = 0
        delta_th = vth * dt
        x += delta_x
        y += delta_y
        # z += delta_z
        # tr = 0
        # tr = 0
        th += delta_th

        # quaternion transfermation
        quat = tf.transformations.quaternion_from_euler(0, 0, th)
        # print(tf.transformations.quaternion_from_euler(0,0,0)) # (0,0,0,1)

        pose = PoseStamped()
        pose.header.stamp = current_time
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        # pose.pose.position.z = z
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]


    # Publish tf (transform between two frames)
    br = tf.TransformBroadcaster()
    # t = geometry_msgs.msg.TransformStamped()
    # br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
    #                  rospy.Time.now(), "odom", "map") # 'odom' -> 'map'; 'map' == 'world'
    t = TransformStamped()
    t.header.frame_id = "map"
    t.header.stamp = current_time
    # t.child_frame_id = "odom"
    t.child_frame_id = pose.header.frame_id
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
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

    # configure the path, adding the current pose to the path
    path_record.header.stamp = current_time
    path_record.header.frame_id = pose.header.frame_id
    path_record.poses.append(pose)

    # limit poses in the path # No necessary, only for simulation path
    if simulation :
        if len(path_record.poses) > 1000:
            path_record.poses.pop(0)

    # Publish the path
    path_pub.publish(path_record)

def main():
    """
    Initial node
    """

    try:
        # Initial a path
        rospy.init_node('PathRecord')

        # creat a publisher path_pub to publish a path with the topic 'trajectory'
        path_pub = rospy.Publisher('trajectory', Path, queue_size=50)

        # initial rate
        rate = rospy.Rate(50)

        # create and initial a path to record the pose
        path_record = Path()

        # Tests if it is running, then updates data
        while not rospy.is_shutdown():
            # Update poses and the path
            pathUpdating(path_pub, path_record)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    # # Initial state of movement
    x, y, z, tr, tp, th = 0, 0, 0, 0, 0, 0

    main()
