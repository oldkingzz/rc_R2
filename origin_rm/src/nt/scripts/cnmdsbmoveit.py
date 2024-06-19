#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw',
                                          Image, self.image_callback)
        self.pos_pub = rospy.Publisher('Ball_Position', Point, queue_size=1)
        self.last_print = rospy.Time.now()
        self.print_interval = rospy.Duration(3)  # print every 3 seconds
        self.focal_length = 2.4  # in mm
        self.real_diameter = 190  # in mm (19 cm)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_blue = numpy.array([100, 43, 46])
        upper_blue = numpy.array([124, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        h, w, d = image.shape
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # estimate the distance based on the size of the ball in the image
            distance = self.estimate_distance(M['m00'])
            distance_a = distance /100
            # estimate the X and Y coordinates based on the position of the ball in the image
            X = (cx - w/2) * distance / self.focal_length
            X_a = X / 100
            Y = (cy - h/2) * distance / self.focal_length
            Y_a = Y /100
            if rospy.Time.now() - self.last_print > self.print_interval:
                # Determine which region the ball is in
                if cx < w/2.5:
                    pos_x = 0
                elif cx > w - w/2.5:
                    pos_x = 2
                else:
                    pos_x = 1
                print("Position: ", "X:", pos_x, "Y",  Y_a,"Z:", distance_a)
                self.last_print = rospy.Time.now()
                # Publish the position
                pos = Point()
                pos.x = pos_x
                pos.y = Y_a
                pos.z = distance_a
                self.pos_pub.publish(pos)

        cv2.imshow("mask", mask)
        cv2.imshow("output", image)
        cv2.waitKey(3)

    def estimate_distance(self, area):
        # estimate the diameter of the ball in the image
        image_diameter = numpy.sqrt(4*area/numpy.pi)
        # estimate the distance based on the size of the ball in the image
        distance = self.focal_length * 2 * self.real_diameter / image_diameter
        return distance

rospy.init_node('follower')
follower = Follower()
rospy.spin()
