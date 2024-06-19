import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf

def get_current_pose():
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
    try:
        (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Failed to get current pose")
        return None, None

def show_pose(client):
    trans, rot = get_current_pose()
    if trans is None or rot is None:
        rospy.logerr("Cannot show pose because current pose is unknown.")
        return

    # 获取当前位置
    current_x, current_y, current_z = trans
    current_qx, current_qy, current_qz, current_qw = rot

    # 打印pose
    rospy.loginfo(f"at: x={current_x}, y={current_y}, orientation=({current_qx}, {current_qy}, {current_qz}, {current_qw})")

def main():
    # 初始化ROS节点
    rospy.init_node('auto_navigate')

    # 创建action客户端，用于与move_base服务器通信
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # 定义目标点列表，每个目标点包含x、y、z坐标和四元数（qx, qy, qz, qw）
    points = [
        (3.16, 0 , 0, 0, 0, 1),
        (3.16, 0.75, 0, 0, 0, 1), 
        (3.16, 1.5, 0, 0, 0, 1),       
        (3.16, -0.75, 0, 0, 0, 1),
        (3.16, -1.5, 0, 0, 0, 1),

    ]

    # 遍历目标点
    for x, y, qx, qy, qz, qw in points:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        # 发送目标点
        rospy.loginfo(f"Going to: x={x}, y={y}, orientation={qx}, {qy}, {qz}, {qw}")
        client.send_goal(goal)
        client.wait_for_result()

        # 检查结果，决定是否继续
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Target reached!")

            # 打印位姿
            show_pose(client)

            rospy.loginfo("Waiting for 10 seconds...")
            rospy.sleep(10)  # 在这里添加停留10秒
        else:
            rospy.loginfo("Failed to reach the target.")
            break  # 如果失败，退出循环

if __name__ == '__main__':
    main() 
