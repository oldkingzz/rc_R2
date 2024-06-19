#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

last_erro = 0
intp = 0
class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                           Twist, queue_size=1)
        self.twist = Twist()
        self.last_print = rospy.Time.now()
        self.last_print1 = rospy.Time.now()

        self.print_interval = rospy.Duration(0.01)  # print every 3 seconds
        self.guwantime = rospy.Duration(1)
        self.white_threshold = 5000  # adjust this value based on your needs
        self.ml = 0
        self.mr = 0

def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = numpy.array([84, 24, 128])
    upper_white = numpy.array([105, 54, 200])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    global last_erro
    h, w, d = image.shape
    search_mid = h // 2  # 定义中间线，将图像分为上下两部分

    # 上半部分
    upper_mask = mask[0:search_mid, 0:w]
    # 下半部分
    lower_mask = mask[search_mid:h, 0:w]

if rospy.Time.now() - self.last_print > self.print_interval:
    # 计算上半部分的质心
    M_upper = cv2.moments(upper_mask)
    if M_upper['m00'] > 0:
        cx_upper = int(M_upper['m10']/M_upper['m00'])
        cy_upper = int(M_upper['m01']/M_upper['m00'])
        cv2.circle(image, (cx_upper, cy_upper), 10, (255, 0, 255), -1)  # 画上半部分质心

    # 计算下半部分的质心
    M_lower = cv2.moments(lower_mask)
    if M_lower['m00'] > 0:
        cx_lower = int(M_lower['m10']/M_lower['m00'])
        cy_lower = int(M_lower['m01']/M_lower['m00'])
        cv2.circle(image, (cx_lower, cy_lower), 10, (0, 255, 0), -1)  # 画下半部分质心

    if cv2.circle:
        # 计算图像中心线和目标指示线中心的距离
        error = (cy_upper - cy_lower)/(cx_lower - cx_upper)
        #erro = cx - w/2 - 15
        d_erro = erro - last_erro
        self.twist.linear.x = 0.18
        threshold = 40  # 误差

        # 判断误差是否在阈值之内
        if abs(erro) <= threshold:
            self.twist.angular.z = 0
            #print('straight')
        else:
            intx = -float(erro) * 0.005 - float(d_erro) * 0.000
            if intx > 0:
                self.twist.angular.z = 0.15
            elif intx < 0:
                self.twist.angular.z = -0.15
                #print('right')
        last_erro = erro
    
    else:
        self.twist.linear.x = 0
        self.twist.angular.z = 0
    self.cmd_vel_pub.publish(self.twist)
    self.last_print = rospy.Time.now()

    # 发布速度命令
    self.cmd_vel_pub.publish(self.twist)

    # 显示图像
    cv2.imshow("mask", mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
