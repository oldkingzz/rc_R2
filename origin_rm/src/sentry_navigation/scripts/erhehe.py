#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   tran_nuc_stm32.py
@Time    :   2024/03/11 08:16:44
@Author  :   33 
@Version :   1.0
@Desc    :   把数据从nuc传到stm32
'''

import rospy
from geometry_msgs.msg import Twist
import struct
import serial
import time
# CRC-8 校验表
CRC08_Table = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

# 定义结构体
class Chassis_Data_Rx:
    def __init__(self, vx, vy, yaw_speed):
        self.vx = vx
        self.vy = vy
        self.yaw_speed = yaw_speed

class T:
    def __init__(self, vel_msg):
        self.vx = vel_msg.linear.x
        self.vy = vel_msg.linear.y
        self.vz = vel_msg.linear.z



# 定义帧头和命令字
HEADER = 0xAA

CMD_ID_CHASSIS_DATA_RX = 0x81

# 打包结构体为字节流

def pack_chassis_data(data):
    return struct.pack('<fff', data.vx, data.vy, data.yaw_speed)

# 计算CRC校验位
def crc8(data):
    crc = 0xff
    for byte in data:
        crc = CRC08_Table[crc ^ byte]
    return crc

# 构建消息
def build_chassis_message(data):
    data_bytes = pack_chassis_data(data)
    length = len(data_bytes) + 4  # 包含帧头、帧长度、命令字和校验位
    crc = crc8(struct.pack('<BBB', HEADER, length, CMD_ID_CHASSIS_DATA_RX) + data_bytes)
    return struct.pack('<BBB', HEADER, length, CMD_ID_CHASSIS_DATA_RX) + data_bytes + struct.pack('<B', crc)

# 连接串口
ser = serial.Serial('/dev/ttyUSB0', 115200)  

def send_yunxin_msg(vx,vy,angular):
    data = Chassis_Data_Rx(vx, vy,angular)
    #rospy.init_node("Sub_node")
    #sub = rospy.Subscriber("/cmd_vel",Twist,Sub_callback,queue_size=10)
    #data = T(Twist)
    message = build_chassis_message(data)
    ser.write(message)
    
def callback_vision(msg):
    status_vision_1 = rospy.get_param("vision")
    status_vision_2 = rospy.get_param("detect_kuang")
    if status_vision_1 or status_vision_2:
        send_yunxin_msg(msg.linear.x , msg.linear.y , msg.angular.z)

        print("Received a vision message!")
        print("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        print("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

def callback_line(msg):
    status_line = rospy.get_param("line_follow")
    if status_line:
        send_yunxin_msg(msg.linear.x , msg.linear.y , msg.angular.z)

        print("Received a line message!")
        print("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        print("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

def callback_nav(msg):
    status_nav_1 = rospy.get_param("nav_1")
    status_nav_2 = rospy.get_param("nav_2")
    if status_nav_1 or status_nav_2:
        send_yunxin_msg(msg.linear.y , msg.linear.x , msg.angular.z)

        print("Received a nav message!")
        print("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        print("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

def listener():
    # 初始化节点
    rospy.init_node('cmd_vel_listener')

    rospy.Subscriber("/cmd_vel_line", Twist, callback_line)
    rospy.Subscriber("/cmd_vel", Twist, callback_nav)
    rospy.Subscriber("/cmd_vel_vision", Twist, callback_vision)

    # 不断循环，直到节点被关闭
    rospy.spin()


if __name__ == "__main__":

    listener()  

