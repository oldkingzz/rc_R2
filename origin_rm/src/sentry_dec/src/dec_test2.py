#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool

while 1:
    rospy.init_node("dec_test")
    pub = rospy.Publisher("/move_wait",Bool,queue_size=1)
    a = Bool()
    a.data = 1
    pub.publish(a)