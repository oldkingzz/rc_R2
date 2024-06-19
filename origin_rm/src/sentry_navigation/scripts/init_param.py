#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node("init_params")

    #1：开启巡线； 0：关闭巡线
    rospy.set_param("line_follow",False)
    #1：巡线到点； 0：巡线未到点
    rospy.set_param("line_follow_get",True)
    #1： 导航1开启，0： 导航1关闭
    rospy.set_param("nav_1",False)
    #1： 导航2开启 ，0： 导航二关闭
    rospy.set_param("nav_2",False)
    #1： 视觉开启 0： 视觉关闭
    rospy.set_param("vision",True)
    #1.  开启框检测 0：关闭框检测
    rospy.set_param("detect_kuang", False)

    rospy.set_param("arm",False)
    rospy.set_param("fan",False)
    rospy.spin()
