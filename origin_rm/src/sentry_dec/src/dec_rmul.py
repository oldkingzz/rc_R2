#! /usr/bin/python3
import rospy
import rospkg
import actionlib
import move_base_msgs.msg as mb
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import yaml
from rm_serial.msg import Referee
import time

def odomcallback(odom: Odometry):
    global odom_last, open_file
    odom_last = odom

    if not open_file:
        open_files()
        open_file = True

def open_files():
    global points,open_file

    f = open(root_path+"/sentry_dec/RMUL.yaml")
    data = yaml.load(f,yaml.SafeLoader)
    points = data["points"]
    for i in points:
        tmp = i.split(" ")
        if tmp[0] == "wall":
            wall_points.append([float(tmp[1]),float(tmp[2])])
        else:
            supply_points.append([float(tmp[1]),float(tmp[2])])
    open_file = 1

def ref_callback(ref: Referee):
    global remain_HP, max_HP, game_progress, rfid_status, HP_supply, supply_begin_hp, supply_time, game_start
    if remain_HP < ref.remain_HP:
        HP_supply -= (ref.remain_HP - remain_HP)
        remain_HP = ref.remain_HP
        supply_begin_hp = remain_HP
        supply_time = time.time()
    elif remain_HP > ref.remain_HP:
        remain_HP = ref.remain_HP
        game_start = 1
    else:
        remain_HP =ref.remain_HP
    game_progress = ref.game_progress
    rfid_status = ref.rfid_status

def pub_goal():
    if not open_file:
        return
    tmp = Bool()
    tmp.data = 0
    pub_wait.publish(tmp)

    if point_now == 0:
        point = wall_points[try_now]
    else:
        point = supply_points[try_now]

    goal = mb.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = float(point[0])
    goal.target_pose.pose.position.y = float(point[1])
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation = odom_last.pose.pose.orientation
    print("pub goal: %f %f",float(point[0]),float(point[1]))

    client.send_goal(goal)

def check_state(*arg):
    global HP_supply, point_now, running, try_now

    if game_progress != 4 and not game_start:
        return
    
    if running:
        state_call()
    
    if HP_supply > 0 and (600 - remain_HP >= 200) and point_now != 1 and not supply_disable:
        point_now = 1
        running = 1
        try_now = 0
        pub_goal()
        return
    
    if point_now == 1 and reach == 1:
        if not supply_disable and time.time() - supply_time > 3:
            print("try next")
            try_next()
            pub_goal()
            return

        if HP_supply == 0 or remain_HP == max_HP or supply_disable:
            pass
        else:
            return

    if not running and reach != 0:
        point_now = 0
        running = 1
        try_now = 0
        pub_goal()

def state_call():
    global point_now,running,reach,supply_time,supply_begin_hp

    result = client.get_state()

    if result == 3:
        print("goal success")
        tmp = Bool()
        tmp.data = 1
        pub_wait.publish(tmp)
        running = False
        reach = point_now
        if reach == 1:
            supply_time = time.time()
            supply_begin_hp = remain_HP
    elif result == 1:
        #print("goal active")
        pass
    elif result == 4:
        print("try next")
        try_next()
        pub_goal()
    else:
        print("goal repub")
        try_next()
        pub_goal()

def try_next():
    global try_now, supply_disable, reach
    try_now += 1
    reach = -1
    if point_now == 0:
        if try_now >= len(wall_points):
            try_now = 0
    else:
        if try_now >= len(supply_points):
            try_now = 0
            supply_disable = 1
            print("supply disable")

def check_goal(*arg):
    global reach
    if reach == -1:
        return
    reach = -1
    pub_goal()

if __name__ == "__main__":

    open_file = 0
    wall_points=[]
    supply_points=[]
    root_path = rospkg.get_ros_paths()[1]

    remain_HP = 600
    max_HP = 600
    game_progress = 0 #4为比赛开始
    rfid_status = 0

    HP_supply = 600
    running = 0
    point_now = 0
    reach = -1

    supply_time = 0
    supply_begin_hp = 0

    try_now = 0
    supply_disable = 0

    game_start = 0

    odom_last = Odometry()
    rospy.init_node("dec_test")
    client = actionlib.SimpleActionClient("move_base", mb.MoveBaseAction)
    sub_ref = rospy.Subscriber("/Referee",Referee,ref_callback)
    client.wait_for_server()
    subodom = rospy.Subscriber("/odom",Odometry,odomcallback)
    pub_wait = rospy.Publisher("/move_wait",Bool,queue_size=5)
    rospy.Timer(rospy.Duration(0.05), check_state)
    rospy.Timer(rospy.Duration(3.0), check_goal)

    rospy.spin()