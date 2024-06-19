#! /usr/bin/python3
import rospy
import rospkg
import actionlib
import move_base_msgs.msg as mbf
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import yaml
import time
from rm_serial.msg import Referee

def odomcallback(odom: Odometry):
    global odom_last, open_file
    odom_last = odom

    if not open_file:
        open_files()
        open_file = True
        
def open_files():
    global repeat,aim_first,steps,step_now,change,first_goal

    f = open(root_path+"/sentry_dec/"+now+".yaml")
    print(now)
    data = yaml.load(f,yaml.SafeLoader)
    repeat = data["repeat"]
    aim_first = data["aim_first"]
    steps = data["steps"]
    change = data["change"]
    step_now = 0
    first_goal = False

def pub_goal(x,y):
    goal = mbf.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation = odom_last.pose.pose.orientation

    client.send_goal(goal)

def step_by_step(*arg):
    global step_now, open_file, running, now, first_goal, begin_time, time_length, time_stop

    if not open_file:
        return
    
    if time_stop:
        tmp = time.time()
        print("wait")
        if tmp - begin_time >= time_length:
            time_stop = False
            step_now += 1
            running = False
            wait_message = Bool()
            wait_message.data = 0
    
    if first_goal:
        check_state()
    
    if not running and step_now + 1 <= len(steps):
        step = steps[step_now].split(" ")
        if step[0] == "point":
            pub_goal(float(step[1]),float(step[2]))
            first_goal = True
        elif step[0] == "time_stop":
            begin_time = time.time()
            time_stop = True
            time_length = float(step[1])
            wait_message = Bool()
            wait_message.data = 1
        running = True

    if step_now + 1 > len(steps):
        if repeat:
            step_now = 0
            first_goal = False
        else:
            open_file = False
            now = change


def check_state(*arg):
    global step_now,running,first_goal

    result = client.get_state()
    print(result)

    if result == 3:
        step_now += 1
        print(step_now)
        running = False
    elif result == 1:
        #continue_last_one()
        pass
    else:
        first_goal = False
        continue_last_one()

        # if result.message == "Action \"move_base\" succeeded!":
        #     step_now += 1
        #     print(step_now)
        #     running = False
        # elif result.message == "Controller canceled":
        #     first_goal = False
        # else:
        #     first_goal = False
        #     continue_last_one()

def cancel():
    client.cancel_goal()

def continue_last_one():
    global running
    running = False

if __name__ == "__main__":

    now = '6_test'
    open_file = False
    repeat = False
    change = None
    aim_first = True
    running = False
    first_goal = False
    time_stop = False
    begin_time = 0
    time_length = 0
    steps = [""]
    step_now = 0
    root_path = rospkg.get_ros_paths()[1]

    odom_last = Odometry()
    rospy.init_node("dec_test")
    client = actionlib.SimpleActionClient("move_base", mbf.MoveBaseAction)
    client.wait_for_server()
    subodom = rospy.Subscriber("/odom",Odometry,odomcallback)
    rospy.Timer(rospy.Duration(0.1), step_by_step)

    rospy.spin()
