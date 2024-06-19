#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np


def moveit_tutorial():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_tutorial_node')

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"  # replace this with the name of your move group
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_named_target('reset')
    move_group.go() #set to home
    time.sleep(5)
    move_group.set_named_target('pick')
    move_group.go() #set to forward
    time.sleep(5)
    move_group.set_named_target('center')
    move_group.go() 
    time.sleep(5)
    move_group.set_named_target('reset')
    move_group.go() 
    time.sleep(5)
    # # Example unit vector (replace with your own values)
    # unit_vector = np.array([ 0.2489-0.04429,-0.2374+0.08251,0.8184-1.045522 ])
 
    # Convert unit vector to quaternion
    # quaternion = unit_vector_to_quaternion(unit_vector)
 
    # print("Quaternion:", quaternion)

    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.w = 0.7650
    # pose_target.orientation.x = 0.4650
    # pose_target.orientation.y = 0.2314
    # pose_target.orientation.z = 0.3807
    # pose_target.position.x = 0.2489
    # pose_target.position.y = -0.2374
    # pose_target.position.z = 0.8184

    # move_group.set_pose_target(pose_target)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    moveit_commander.roscpp_shutdown()
# def unit_vector_to_quaternion(unit_vector):
#     w = np.sqrt(1 + unit_vector[0] + unit_vector[1] + unit_vector[2]) / 2
#     x = (unit_vector[2] - unit_vector[1]) / (4 * w)
#     y = (unit_vector[0] - unit_vector[2]) / (4 * w)
#     z = (unit_vector[1] - unit_vector[0]) / (4 * w)
#     return np.array([w, x, y, z])

if __name__ == '__main__':
    try:
        moveit_tutorial()
    except rospy.ROSInterruptException:
        pass
#   homeposition:  
#   x: 0.04429299898457456
#   y: -0.0825158909879856
#   z: 1.045522998942463