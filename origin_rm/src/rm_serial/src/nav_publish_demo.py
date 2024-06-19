import rospy
from rm_serial.msg import chassis,pnpSolution,Referee
def velocity_publisher():
    # ROS节点初始化
    rospy.init_node('person_publisher', anonymous=True)

    # 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    person_info_pub = rospy.Publisher('/move_cmd', chassis, queue_size=10)

    #设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        # 初始化learning_topic::Person类型的消息
        person_msg = chassis()
        person_msg.x_speed = 2;
        person_msg.y_speed = 2;
        person_msg.rotate_speed = 3;
        person_msg.yaw = 4;
        person_msg.pitch = 5;


        # 发布消息
        person_info_pub.publish(person_msg)

        # 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass

