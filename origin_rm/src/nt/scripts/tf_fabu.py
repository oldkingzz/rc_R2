#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import  tf2_ros
#from tf_conversions import transformations
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
import math
class TFPublish():
    def __init__(self):
        rospy.init_node('tf_pub')
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size = 1)
        rospy.loginfo("tf_pub python demo")
        static_shexiangtou =  TransformStamped()
        static_shexiangtou.header.stamp = rospy.Time.now()

        static_shexiangtou.header.frame_id = "revolute_Link"
        static_shexiangtou.child_frame_id = "camera_link"
        static_shexiangtou.transform.translation.x = 0.16521
        static_shexiangtou.transform.translation.y = 0
        static_shexiangtou.transform.translation.z = 0.41845
        static_shexiangtou.transform.rotation.x = 0
        static_shexiangtou.transform.rotation.y = 0
        static_shexiangtou.transform.rotation.z = 0
        static_shexiangtou.transform.rotation.w = 1

        static_camera_br = tf2_ros.StaticTransformBroadcaster()
        static_camera_br.sendTransform(static_shexiangtou)


        
if __name__ == '__main__':
    print("1")
    try:
        tf_pub = TFPublish()
        rospy.spin()
    except rospy.ROSInterruptException():
        rospy.loginfo("TF publisher is shut down")