#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from rm_serial.msg import chassis
from std_msgs.msg import Bool
from rm_serial.msg import Referee

def send(origin_data:Twist):
    rospy.loginfo("##########################%f %f",origin_data.linear.x,origin_data.angular.z)
    move_cmd = chassis()
    move_cmd.x_speed = origin_data.linear.x
    move_cmd.y_speed = origin_data.linear.y
    move_cmd.yaw = origin_data.angular.z * 1000
    move_cmd.rotate_speed=5*1000
    move_cmd.pitch = 1*1000
    if game_progress == 4:
        pub_move.publish(move_cmd)
    else:
        move_cmd.x_speed = 0
        move_cmd.y_speed = 0
        move_cmd.yaw = 0
        move_cmd.rotate_speed=0
        move_cmd.pitch = 0
        pub_move.publish(move_cmd)
    #gimbal_send.send_nav(origin_data.linear.x,origin_data.linear.y,origin_data.angular.z*1000)

def wait_callback(wait_message: Bool):
    move_cmd = chassis()
    if wait_message.data == 1:
        move_cmd.x_speed = 0
        move_cmd.y_speed = 0
        move_cmd.yaw = 0
        move_cmd.rotate_speed=2.51*1000
        move_cmd.pitch = 1*1000
    else:
        move_cmd.x_speed = 0
        move_cmd.y_speed = 0
        move_cmd.yaw = 0
        move_cmd.pitch = 1*1000
    pub_move.publish(move_cmd)

def ref_callback(ref:Referee):
    global game_progress
    game_progress = ref.game_progress
    

game_progress = 0

rospy.init_node("send_cmd_vel")
rospy.Subscriber("/cmd_vel",Twist,send)
rospy.Subscriber("/move_wait",Bool,wait_callback)
sub_ref = rospy.Subscriber("/Referee",Referee,ref_callback)
pub_move = rospy.Publisher("/move_cmd",chassis,queue_size=1)
rospy.spin()