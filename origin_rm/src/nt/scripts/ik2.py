#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
def init_moveit():
    # Initialize move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize ROS node
    rospy.init_node('moveit_ik_demo')

    # Initialize the arm group
    arm = moveit_commander.MoveGroupCommander('arm')

    return arm

def setup_arm(arm):
    # Get the name of the end effector link
    end_effector_link = arm.get_end_effector_link()

    # Set the reference frame for target poses
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

    # Set the tolerance for position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.05)
    arm.set_named_target('home')
    arm.go()
    return end_effector_link, reference_frame

def move_to_pose(arm, end_effector_link, reference_frame,msg):
    # Set the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()     
    if msg.x == 0:
        arm.set_named_target('left')
        arm.go()
    elif msg.x == 1:
        arm.set_named_target('center')
        arm.go()
    elif msg.x == 2:
        arm.set_named_target('right')
        arm.go()
    print(msg.x)
    arm.set_named_target('sb')
    arm.go()
    # Set the start state to the current state

def return_to_home(arm):
    # Return the arm to the home position
    arm.set_named_target('home')
    arm.go()

def shutdown_moveit():
    # Shutdown MoveIt and exit the script
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
def callback(msg):
    arm = init_moveit()
    end_effector_link, reference_frame = setup_arm(arm)
    move_to_pose(arm, end_effector_link, reference_frame,msg)
    #return_to_home(arm)
    time.sleep(5)
if __name__ == "__main__":
    arm = init_moveit()
    time.sleep(6)
    rospy.Subscriber("/Ball_Position", Point, callback)
    
    rospy.spin()
    
