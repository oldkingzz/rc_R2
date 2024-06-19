#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Bool
import numpy as np


class MoveItTutorial:

    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_tutorial_node')

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"  # replace this with the name of your move group
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.reach_sub = rospy.Subscriber('reach', Bool, self.reach_callback)

        # Add a state variable
        self.pose_executed = False

    def reach_callback(self, msg):
        if msg.data and not self.pose_executed:
            self.execute_pose()
            self.pose_executed = True

    def execute_pose(self):
        self.move_group.set_named_target('reset')
        self.move_group.go()  # set to home
        time.sleep(5)
        self.move_group.set_named_target('pick')
        self.move_group.go()  # set to forward
        time.sleep(5)
        self.move_group.set_named_target('center')
        self.move_group.go()
        time.sleep(5)

    def reset_pose(self):
        self.move_group.set_named_target('reset')
        self.move_group.go()  # set to home
        time.sleep(5)

        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()


if __name__ == '__main__':
    try:
        tutorial = MoveItTutorial()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
