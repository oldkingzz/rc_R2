#!/usr/bin/env python3
"""#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class TFListen():
    def __init__(self):
        rospy.init_node('tf_listen')
        rospy.loginfo("tf_listen Python demo")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)

        self.point_sub = rospy.Subscriber("/ball_coordinates", PointStamped, self.point_callback)
        self.point_pub = rospy.Publisher("/ball_coordinates_base_link", PointStamped, queue_size=10)

        rospy.spin()

    def point_callback(self, data):
        try:
            # Transform PointStamped from camera_link to base_link
            point_base_link = self.tfBuffer.transform(data, "base_link", rospy.Duration(1.0))
            # Publish the transformed PointStamped
            self.point_pub.publish(point_base_link)
            rospy.loginfo("Position in base_link: x=%f, y=%f, z=%f",
                          point_base_link.point.x,
                          point_base_link.point.y,
                          point_base_link.point.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()
            return

    def listen_recent(self):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('base_link', 'camera_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

if __name__ == "__main__":
    TFListen()

"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point
class TFListen():
    def __init__(self):
        rospy.init_node('tf_listen')
        rospy.loginfo("tf_listen Python demo")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)

        self.point_sub = rospy.Subscriber("/ball_coordinates", Point, self.point_callback)
        self.point_pub = rospy.Publisher("/ball_coordinates_base_link", PointStamped, queue_size=10)

        rospy.spin()

    def point_callback(self, data):
    # 4. 组织被转换的坐标点 头文件3
        ps = tf2_geometry_msgs.PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "camera_link"
        ps.point.x = data.x
        ps.point.y = data.y
        ps.point.z = data.z

        # 5. 转换逻辑实现，调用tf封装的算法
        rate = rospy.Rate(1)
        current_time = rospy.Time.now()
        # 使用try防止
        try:
            # 转换实现
            transform = self.tfBuffer.lookup_transform("base_link", 
                                                   "camera_link", 
                                                   current_time, 
                                                   rospy.Duration(1.0))
            ps_out = self.tfBuffer.transform(ps,"base_link")
            # 6. 输出结果
            rospy.loginfo("转换后的坐标:(%.2f,%.2f,%.2f),参考的坐标系: %s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps_out.header.frame_id)
            self.point_pub.publish(ps_out)
        except Exception as e:
            rospy.logwarn("错误提示:%s",e) 
            
            rate.sleep()
if __name__ == "__main__":
    TFListen()