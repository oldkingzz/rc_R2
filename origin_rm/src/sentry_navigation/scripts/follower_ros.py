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
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_follower',
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
        # lower_white = numpy.array([0, 0, 221])
        # upper_white = numpy.array([180, 30, 255])
        lower_white = numpy.array([84,24,128])
        upper_white = numpy.array([105,54,200])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        global last_erro
        h, w, d = image.shape
        search_top = h // 2  # 

        # Define regions A, B, C
        A_mask = mask[0:search_top, 0:w//3]  # upper left
        B_mask = mask[0:search_top, w//3:2*w//3]  # upper middle
        C_mask = mask[0:search_top, 2*w//3:w]  # upper right
        E_mask = mask[search_top:h, 0:w//3]  # upper left
        F_mask = mask[search_top:h, w//3:2*w//3]  # upper middle
        G_mask = mask[search_top:h, 2*w//3:w]  # upper right
        # D_mask = mask[0:h, 0:w]
        # Check the type of intersection
        A_white = cv2.countNonZero(A_mask)
        B_white = cv2.countNonZero(B_mask)
        C_white = cv2.countNonZero(C_mask)

        # if rospy.Time.now() - self.last_print > self.print_interval:
        #     if A_white > self.white_threshold and B_white > self.white_threshold and C_white < self.white_threshold:
        #         # self.ml += 1
        #         print("Left")
        #         # if self.ml == 2:
        #         #     print("Turn Left")
        #         #     self.ml = 0  # reset ml
        #         if rospy.Time.now() - self.last_print > self.guwantime: #用的同一套时间戳
        #             self.twist.linear.x = 0.18
        #             self.twist.angular.z = 0.12
        #         self.last_print = rospy.Time.now()
        #     elif B_white > self.white_threshold and C_white > self.white_threshold and A_white < self.white_threshold:
        #         # self.mr += 1
        #         print("Right")
        #         if rospy.Time.now() - self.last_print1 > self.guwantime: #用的同一套时间戳
        #             self.twist.linear.x = 0.18
        #             self.twist.angular.z = 0.12
        #             self.last_print1 = rospy.Time.now()
        #         # if self.mr == 2:
        #         #     print("Turn Right")
        #         #     self.mr = 0  # reset mr
        #         self.last_print = rospy.Time.now()
        #     elif B_white > self.white_threshold and A_white < self.white_threshold and C_white < self.white_threshold:
        #         print("|")
        #         M = cv2.moments(mask)  
        #         if M['m00'] > 0:
        #             cx = int(M['m10']/M['m00'])
        #             cy = int(M['m01']/M['m00'])
        #             cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
        #             #cv2.circle(image, (cx-75, cy), 10, (0, 0, 255), -1)
        #             #cv2.circle(image, (w/2, h), 10, (0, 255, 255), -1)
        #             if cv2.circle:
        #             # 计算图像中心线和目标指示线中心的距离
        #                 erro = cx - w/2-15
        #                 d_erro=erro-last_erro
        #                 self.twist.linear.x = 0.18
        #                 if erro!=0:
        #                     self.twist.angular.z = -float(erro)*0.005-float(d_erro)*0.000
        #                 else :
        #                     self.twist.angular.z = 0
        #                 last_erro=erro
        #         else:
        #             self.twist.linear.x = 0
        #             self.twist.angular.z = 0
        #         self.cmd_vel_pub.publish(self.twist)
        #         self.last_print = rospy.Time.now()
        #     elif B_white > self.white_threshold and C_white > self.white_threshold and A_white > self.white_threshold:
        #         print("十 ")
        #         global intp 
        #         intp += 1
        #         if rospy.Time.now() - self.last_print1 > self.guwantime: #用的同一套时间戳
        #             self.twist.linear.x = 0.18
        #             self.twist.angular.z = 0.12
        #             self.last_print1 = rospy.Time.now()
        #         self.last_print = rospy.Time.now()
        # cv2.imshow("mask", mask)
        # cv2.imshow("output", image)
        # cv2.waitKey(3)

        if rospy.Time.now() - self.last_print > self.print_interval:
            M = cv2.moments(mask)  
            if M['m00'] > 0:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
                #cv2.circle(image, (w/2, h/2), 10, (255, 255, 255), -1)
                #cv2.circle(image, (cx-75, cy), 10, (0, 0, 255), -1)
                cv2.circle(image, (int(w/2), int(h/2)), 10, (0, 255, 255), -1)
                if cv2.circle:
                # 计算图像中心线和目标指示线中心的距离
                    erro = cx - w/2
                    d_erro = erro - last_erro
                    self.twist.linear.x = 0.3
                    threshold = 10  # 误差

                    # 判断误差是否在阈值之内
                    if abs(erro) <= threshold:
                        self.twist.angular.z = 0
                        print('stright %f'%abs(erro))
                    else:
                        intx = -float(erro) * 0.005 - float(d_erro) * 0.000
                        if intx > 0:
                            self.twist.angular.z = 0.05
                            print('left %f'%abs(erro))
                        elif intx < 0:
                            self.twist.angular.z = -0.05
                            print('right %f'%abs(erro))
                    last_erro = erro
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            self.last_print = rospy.Time.now()
        cv2.imshow("mask", mask)
        cv2.imshow("output", image)
        cv2.waitKey(3)

line_get = True             
rospy.init_node('follower')
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    line_follow = rospy.get_param("line_follow")
    if(line_follow == True):
        print(2)
        follower = Follower()
        rospy.sleep(15)
        if(line_get == True):
            rospy.set_param("line_follow",False)
            rospy.set_param("line_follow_get",True)

rospy.spin()
