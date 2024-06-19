#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node("test_param")
    rospy.set_param("line_follow",False)
    a = rospy.get_param("line_follow",True)
    rospy.loginfo("%s",a)
    rospy.spin()
