import rospy
import numpy as np
from std_msgs.msg import String, Bool, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point, PoseArray, Quaternion
from ros_pp_prystine.msg import ControlInputData, ControlOutputData, TrafficLight, Dms

import numpy as np
import _thread
import time

from cubic_spline_planner import *

L = 3.170
pp_k = 0.01
pp_look_ahead = 15

max_steer = 1.1
accel_limit = 0.3
accel = 0
start_accel_limit = 0.1

sem_passed = False
driving_info_available = True
static_throttle = 0.2

path_to_follow = 0

speed_target = 10
stop = False

# car sim state
state = { "x": 0, "y": 0, "speed": 0, "yaw": 0, "accel": 0, "brake": 0}

#sem state
sem_state = { "dist": 0, "status": 0, "time_to_change": 0}

#0 ok, 1 emergency, should stop
dms_status = 0

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def pp_step(target, speed):
    alpha = math.atan2(target[1], target[0])
    #Lf = pp_k * speed + pp_look_ahead
    #delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
    delta = alpha
    if delta > max_steer:
        delta = max_steer
    if delta < -max_steer:
        delta = -max_steer
    return delta

def build_path_msg(xs, ys):
    p_msg = Path()
    p_msg.header.frame_id = "map"
    
    for i in range(len(xs)):
        p = PoseStamped()
        p.pose.position.x = xs[i] 
        p.pose.position.y = ys[i]
        p_msg.poses.append(p)
    return p_msg

def load_flag_path(file_path):
    file = open(file_path, "r")
    
    xs = []
    ys = []

    while(file.readline()):
        lx = file.readline()
        ly = file.readline()
        xs.append( float(lx.split(" ")[-1]) )
        ys.append( float(ly.split(" ")[-1]) )

    return xs, ys

def input_callback(data):

    state["x"] = data.world_position.position.x
    state["y"] = data.world_position.position.y
    state["yaw"] = data.world_position.orientation.z
    state["speed"] = data.speed
    state["accel"] = data.accelX
    state["brake"] = data.brakePedalSts

def sem_callback(data):

    sem_state["sem_status"] = data.sem_status
    sem_state["time_to_change"] = data.time_to_change
    sem_state["distance"] = calc_distance([data.position.position.x, data.position.position.y], state["x"], state["y"])
    #sem_state["distance"] = data.distance

def dms_callback(data):

    dms_status = data.dms_status
        
def calc_distance(pose, path_pose):
	return (pose[0] - path_pose[0])**2 + (pose[1] - path_pose[1])**2

def spin():
    print ("spin start")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ros_pp')

    #Subscribers
    rospy.Subscriber("/ego_data", ControlInputData, input_callback)
    rospy.Subscriber("/sem_data", TrafficLight, sem_callback)
    rospy.Subscriber("/dms_data", Dms, dms_callback)

    #Publishers
    pub_drive = rospy.Publisher('control_output_data', ControlOutputData, queue_size=1)
    '''pub_path = rospy.Publisher('path', Path, queue_size=1)
    pub_pose = rospy.Publisher('/actual_pos', PoseStamped, queue_size=1)
    pub_goal = rospy.Publisher('/target_pos', PoseStamped, queue_size=1)
    pub_orig = rospy.Publisher('/orig_pos', PoseStamped, queue_size=1)'''

    _thread.start_new_thread(spin, ())

    max_tracking_error = 0
    path_tracking_error = 0

    last_pose = []
    sampling_distance = 0.5
    i_error = 0

    # spline to follow
    xs_ref, ys_ref = load_flag_path("path.trj")
    path_ref = Spline2D(xs_ref, ys_ref)

    xs_right, ys_right = load_flag_path("path_right.trj")
    path_right = Spline2D(xs_right, ys_right)

    s = np.linspace(0, path.s[-1]-0.0001, 2000)
    s_x = [ path.calc_position(t)[0] for t in s ]
    s_y = [ path.calc_position(t)[1] for t in s ]

    '''p_msg = build_path_msg([ path.calc_position(t)[0] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ],
                    [ path.calc_position(t)[1] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ])'''

    print("START")
    r = rospy.Rate(1000) 
    while not rospy.is_shutdown():
        t2 = time.time()

        print("---------------- PP IS WORKING ----------------")

        if(sem_state["distance"] < 1):
            sem_passed = True

        if(path_to_follow == 0):
            path = path_ref
        else:
            path = path_right

        # get pose and update spline coords
        pose = state["x"], state["y"]
        path.update_current_s(pose)

        # get pose on spline
        path_pose = path.calc_position(path.cur_s)

        # compute target speed
        #p_pos =  path.cur_s + L*state["speed"]*1.8
        p_pos =  path.cur_s + 0.1 * pp_k * state["speed"] + pp_look_ahead

        '''accel =+ 0.6 * (speed_target - state["speed"])
        if accel < -1: accel = -1
        if accel > 1: accel = 1'''

        speed = speed_target
        # frena se ricevo segnale dms/ostacolo
        if dms_status:
            stop = True

        if stop:
            if(state["speed"] < 3.0):
                speed = -0.5
            else:
                speed = 0.0

        # Start accelerating in a smooth way
        #accel = min(accel, start_accel_limit)   # limit speed at start
        '''if(start_accel_limit > 0 and start_accel_limit<accel_limit):
            start_accel_limit += 1.0/40.0
            print "START accel: ", start_accel_limit'''

        # get target pose
        s_pos = path.cur_s + pp_k * state["speed"] + pp_look_ahead
        trg = path.calc_position(s_pos)
        trg = [ trg[0], trg[1] ]
        #print(trg)
        local_straight = [trg[0] - state["x"], trg[1] - state["y"]]
        local_trg = [local_straight[0]*math.cos(state["yaw"]) + local_straight[1]*math.sin(state["yaw"]), \
            -local_straight[0]*math.sin(state["yaw"]) + local_straight[1]*math.cos(state["yaw"])]
        print("----------------------------",local_trg)        
        steer = pp_step(local_trg, state["speed"]) #* 12.6

        '''print "STEER COMMAND: ", steer
        print "CURRENT SPEED", state["speed"]
        print "TARGET SPEED", speed_target
        print "CURRENT ACCEL", accel
        print "LookAhead distance ----> ",  pp_k * state["speed"] + pp_look_ahead'''

        # actuate
        msg = ControlOutputData()
        h = Header()
        now = rospy.Time.now()
        h.stamp = now
        msg.header = h
        msg.steerAngle = steer
        msg.speed = speed
        
        pub_drive.publish(msg)

        #r.sleep()
        tot_time = time.time() - t2
        print("TIME: ", tot_time*1000)
