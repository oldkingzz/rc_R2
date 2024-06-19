#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import rospy
import moveit_commander
from std_msgs.msg import Bool

def main():
    rospy.init_node('fan_open')
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        fan = rospy.get_param("fan")
        if(fan == True):
            print(2)
            handaoshan_pub = rospy.Publisher('handaoshan', Bool, queue_size=10)
            handaoshan_pub.publish(True)  # Publish True before executing pose
            #handaoshan_pub.publish(False)  # Publish True before executing pose
            print(3)
        else:
            handaoshan_pub = rospy.Publisher('handaoshan', Bool, queue_size=10)
            handaoshan_pub.publish(False)  # Publish True before executing pose

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
