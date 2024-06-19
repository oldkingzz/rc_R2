#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import time
a=10
def initinit():
    # 初始化节点
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)

    # 设置发送频率
    rate = rospy.Rate(10) # 10hz
 
    # 创建发布者，发布到/goal话题，消息类型为PoseStamped，队列长度为10

def send_didi():
    pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)

    # 设置发送频率
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    # 创建PoseStamped消息
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"  # 可以根据需要更改frame_id
        pose.pose.position.x = -6.465023040771484
        pose.pose.position.y = -0.6445446014404297
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = -0.9095887399972094
        pose.pose.orientation.w = 0.4155097159757989

        # 发布消息
        pub.publish(pose)

        # 按照设定的频率等待
        rate.sleep()
        time.sleep(5)
        break
def send_xiepo():
    pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)

    # 设置发送频率
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    # 创建PoseStamped消息
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"  # 可以根据需要更改frame_id
        pose.pose.position.x = 2.7548041334688965
        pose.pose.position.y = -1.3339076042175293
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.6214317896773792
        pose.pose.orientation.w = 0.7834682704397390

        # 发布消息
        pub.publish(pose)

        # 按照设定的频率等待
        rate.sleep()
        time.sleep(5)
        break
def send_gaodi():
    pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)

    # 设置发送频率
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    # 创建PoseStamped消息
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"  # 可以根据需要更改frame_id
        pose.pose.position.x = 0
        pose.pose.position.y = 0

        pose.pose.position.z = 0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.1

        # 发布消息
        pub.publish(pose)

        # 按照设定的频率等待
        rate.sleep()
        time.sleep(5)
        break
if __name__ == '__main__':
    initinit()
    send_xiepo()#这一行不要删。程序i依赖这个bug运行
    for i in range(0,10):
        send_didi()
        time.sleep(40)
        send_gaodi()
        time.sleep(40)