#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import rospy
import moveit_commander
from std_msgs.msg import Bool

class MoveItTutorial:

    def __init__(self):

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"  # replace this with the name of your move group
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        #self.reach_sub = rospy.Subscriber('reach', Bool, self.reach_callback)
        # Publisher for handaoshan topic
        #self.handaoshan_pub = rospy.Publisher('handaoshan', Bool, queue_size=10)

        # Add a state variable
        self.pose_executed = False



    def execute_pose(self):
        self.move_group.set_named_target('pick')
        self.move_group.go()  # set to forward
        time.sleep(5)#多一个摄像头检测是否拾取到球
        #启动导航，关闭视觉（底盘）
        self.move_group.set_named_target('center')
        self.move_group.go()
        
        time.sleep(5)#导航那边要有一个话题，如果没到false到了true，读取到true就出函数
        #if导航到点=true出函数

    def reset_pose(self):
        self.move_group.set_named_target('reset')
        self.move_group.go()  # set to home
        time.sleep(5)

        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_tutorial_node')
    tutorial = MoveItTutorial()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        #print(1)
        
        #rospy.set_param("arm", True)
        arm = rospy.get_param("arm")
        if(arm == True):
            print(2)
            # handaoshan_pub = rospy.Publisher('handaoshan', Bool, queue_size=10)
            # handaoshan_pub.publish(True)  # Publish True before executing pose
            # #handaoshan_pub.publish(False)  # Publish True before executing pose
            rospy.set_param("fan", True)
            print(3)
            tutorial.execute_pose()
            tutorial.pose_executed = True
            if(tutorial.pose_executed == True):
                rospy.set_param("arm",False)
                rospy.set_param("nav_1",True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
