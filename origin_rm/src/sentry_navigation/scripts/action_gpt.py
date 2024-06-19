#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf
from std_msgs.msg import Bool

# 全局变量，用于记录导航成功与否
goal_reached = False

# 定义目标点列表，每个目标点包含x、y、z坐标和四元数（qx, qy, qz, qw）
points = [
    (3.16, 0 , 0, 0, 0, 1),
    (3.16, 0.75, 0, 0, 0, 1), 
    (3.16, 1.5, 0, 0, 0, 1),       
    (3.16, -0.75, 0, 0, 0, 1),
    (3.16, -1.5, 0, 0, 0, 1),
    (0, 0, 0, 0 ,0 ,1)
]

def get_current_pose():
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
    try:
        (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Failed to get current pose")
        return None, None

def show_pose(client):
    trans, rot = get_current_pose()
    if trans is None or rot is None:
        rospy.logerr("Cannot show pose because current pose is unknown.")
        return

    # 获取当前位置
    current_x, current_y, current_z = trans
    current_qx, current_qy, current_qz, current_qw = rot

    # 打印pose
    rospy.loginfo(f"at: x={current_x}, y={current_y}, orientation=({current_qx}, {current_qy}, {current_qz}, {current_qw})")

def navigate_to_goal(index):
    global goal_reached
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    if 0 <= index < len(points):
        x, y, qx, qy, qz, qw = points[index]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        # 发送目标点
        rospy.loginfo(f"Going to: x={x}, y={y}, orientation={qx}, {qy}, {qz}, {qw}")
        client.send_goal(goal)
        client.wait_for_result()

        # 检查结果，决定是否更新goal_reached变量
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Target reached!")
            goal_reached = True
        else:
            rospy.loginfo("Failed to reach the target.")
            goal_reached = False
    else:
        rospy.logwarn(f"Invalid index: {index}")


def main():
    # 初始化ROS节点
    global goal_reached

    rospy.init_node('auto_navigate')
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        print("总体启动")
        nav_1 = rospy.get_param("nav_1")
        nav_2 = rospy.get_param("nav_2")

        if(nav_1 == True and nav_2 == False):
            print("nav_1启动")
            navigate_to_goal(2)
            rospy.sleep(5)
            rospy.set_param("nav_1",False)
            # rospy.set_param("detect_kuang",True)
        if(goal_reached == True):
            print("喊道山")
            rospy.set_param("fan", False)
            rospy.set_param("nav_2",True)
            goal_reached = False
            
        if(nav_1 == False and nav_2 == True):
            print("nav_2启动")
            navigate_to_goal(5)
            rospy.sleep(2)
            rospy.set_param("nav_2",False)
            rospy.set_param("vision",True)       
        rate.sleep()

if __name__ == '__main__':
    main() 
