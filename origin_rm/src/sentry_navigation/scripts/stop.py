#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def zero_velocity_publisher():
    # 创建一个名为'zero_velocity_publisher'的新节点
    rospy.init_node('zero_velocity_publisher')

    # 创建一个名为'cmd_vel'的发布者，消息类型为Twist，队列长度为10
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # 设置发布频率为10hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # 创建一个Twist消息对象
        zero_vel_msg = Twist()

        # 设置线速度和角速度都为0
        zero_vel_msg.linear.x = 0
        zero_vel_msg.linear.y = 0
        zero_vel_msg.linear.z = 0
        zero_vel_msg.angular.x = 0
        zero_vel_msg.angular.y = 0
        zero_vel_msg.angular.z = 0

        # 发布消息
        pub.publish(zero_vel_msg)

        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        zero_velocity_publisher()
    except rospy.ROSInterruptException:
        pass
