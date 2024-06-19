#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def get_current_pose(listener):
    try:
        listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Failed to get current pose")
        return None, None

def pose_changed(prev_trans, prev_rot, trans, rot):
    return prev_trans != trans or prev_rot != rot

def print_pose(trans, rot):
    if trans and rot:
        rospy.loginfo(f"Position: x={trans[0]}, y={trans[1]}, z={trans[2]}")
        rospy.loginfo(f"Orientation: x={rot[0]}, y={rot[1]}, z={rot[2]}, w={rot[3]}\n")

def main():
    rospy.init_node('pose_to_console')
    listener = tf.TransformListener()

    prev_trans, prev_rot = None, None

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        trans, rot = get_current_pose(listener)
        if trans and rot and pose_changed(prev_trans, prev_rot, trans, rot):
            print_pose(trans, rot)
            prev_trans, prev_rot = trans, rot
        rate.sleep()

if __name__ == '__main__':
    main()
