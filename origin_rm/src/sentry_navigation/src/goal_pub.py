#! /usr/bin/python3
import rospy
import actionlib
import move_base_msgs.msg as mb
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Bool

def pub_point(point: PoseStamped):
    global goal_send
    goal = mb.MoveBaseGoal()
    goal.target_pose = point
    client.send_goal(goal)
    goal_send = 1

def check_state(*arg):
    global goal_send
    state = client.get_state()
    if not goal_send:
        return
    if state == 0:
        print("The goal has yet to be processed by the action server")
    elif state == 1:
        print("The goal is currently being processed by the action server")
    elif state == 2:
        print("The goal received a cancel request after it started executing and has since completed its execution (Terminal State)")
    elif state == 3:
        print("The goal was achieved successfully by the action server (Terminal State)")
        goal_send = 0
    elif state == 4:
        print("The goal was aborted during execution by the action server due to some failure (Terminal State)")
        goal_send = 0
    elif state == 5:
        print("The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)")
    elif state == 6:
        print("The goal received a cancel request after it started executing and has not yet completed execution")
    elif state == 7:
        print("The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled")
    elif state == 8:
        print("The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)")
    else:
        print("An action client can determine that a goal is LOST. This should not be sent over the wire by an action server")

if __name__ == "__main__":

    rospy.init_node("goal_pub")
    sub_point = rospy.Subscriber("/goal",PoseStamped,pub_point)
    pub_cmd_repub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    client = actionlib.SimpleActionClient("move_base", mb.MoveBaseAction)
    client.wait_for_server()
    rospy.Timer(rospy.Duration(0.1),check_state)
    goal_send = 0

    rospy.spin()