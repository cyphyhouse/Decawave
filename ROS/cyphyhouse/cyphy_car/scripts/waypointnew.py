#!/usr/bin/env python
#NOTE: To run, SSH in with -X flag so that pygame console can be run.
import pygame as pg
import sys
import rospy
import subprocess
import os
import math
import numpy as np
import time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from multiprocessing import Process, Value, Array

FPS = 100  # frames per second: number of messages we wish to publish per sec
COMMUNICATOR_SLEEP_MILLIS = .001    # period for sending/receiving waypoint/acks.
STOP = 0
DELTA_DIRECTION = 0.05
DELTA_SPEED_DRIVE = 0.25
EPSILON_RADIUS = 0.25
EPSILON_ANGLE = .15
INITIAL_RUN_LOOPS = 5
# bash command for bagging data using this topic.
#BASH_CMD = "rosbag record drive_parameters"
#BAG_DIR = "/media/ubuntu/9C33-6BBD1/bagfiles"
#CURR_DIR = "/home/ubuntu/catkin_ws/src/week1tutorial/src"
#os.chdir(BAG_DIR) 
#process = subprocess.Popen(BASH_CMD, shell=True) #stdout=subprocess.PIPE)
#output, error = process.communicate()
#os.chdir(CURR_DIR)

pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=10) 

points = [(-1.57, -.63), (2.43, 1.80), (-1.66, 2.2), (3.31, -0.2), \
        (-1.57, -.63), (2.43, 1.80), (-1.66, 2.2), (3.31, -0.2), \
        (-1.57, -.63), (2.43, 1.80), (-1.66, 2.2), (3.31, -0.2)]
pos = [0, 0]


def deca_callback(data):
    pos[0] = data.x
    pos[1] = data.y

def vicon_callback(data):
    pos[0] = data.pose.position.x
    pos[1] = data.pose.position.y
    #print(pos[0])
    #print(pos[1])	

def get_distance(pos_prev, pos_curr):
    return np.sqrt((pos_curr[0] - pos_prev[0]) ** 2 + (pos_curr[1] - pos_prev[1]) ** 2)


def get_angle_between_3_pts(center, waypoint, next_pos):
    n_waypoint = waypoint[0] - center[0], waypoint[1] - center[1]
    n_next_pos = next_pos[0] - center[0], next_pos[1] - center[1]
    theta = np.arctan2(n_waypoint[1], n_waypoint[0]) - np.arctan2(n_next_pos[1], n_next_pos[0])
    return theta


def get_next_loc(curr, prev):
    x_step = curr[0] - prev[0]
    y_step = curr[1] - prev[1]
    return curr[0] + x_step, curr[1] + y_step


def drive():
    
    pg.display.set_mode((400, 300))   
  
    reached = [False]
    waypoint = [0, 0]
    single_ack_entry_flag = [False]
    #sub = rospy.Subscriber('decaPos', Point, deca_callback)
    sub = rospy.Subscriber('vrpn_client_node/f1car/pose', PoseStamped, vicon_callback)   
    waypoint_listener = rospy.Subscriber("Waypoint_bot0", Point, waypoint_callback,
            (waypoint, single_ack_entry_flag))
    ack_publisher = rospy.Publisher("Reached", String, queue_size=1) 

    pg.init()
    clock = pg.time.Clock()
    speed = STOP
    direction = 0
    # states of keys: 0 indicates up, 1 is down.
    kUp = kDown = kLeft = kRight = 0
    run = True
    down = False
    cycles_since_stop = 0
    accelerate = False
    stop_cmd = False
    path = [] 
    curr_delta = DELTA_SPEED_DRIVE
    for i in range(INITIAL_RUN_LOOPS):
        x = pos[0]
        y = pos[1]
        path.append((x,y))
        x_prev = x
        y_prev = y
        kUp = 1
        kDown = 0
        #clock.tick(FPS)
	for event in pg.event.get():
	    if event.type == pg.KEYDOWN:
		key = event.key
		if key == pg.K_q:
		    stop()
		    return

    #stop()
    time.sleep(0.5)
    #waypoint = points.pop(0)
    x_prev, y_prev = path[-1]
    d_prev = 0.0
    a_prev = 0.0
    d_integral = 0.0
    a_integral = 0.0

    while run:
        x = pos[0]
        y = pos[1]
        path.append((x,y))
        curr_loc = path[-1]
        prev_loc = path[-2]
        next_loc = get_next_loc(curr_loc, prev_loc)
        #d_target = get_distance(waypoint, curr_loc)
        a_error = get_angle_between_3_pts(center=curr_loc, waypoint=waypoint, next_pos=next_loc)
        if a_error > math.pi:
            a_error = a_error - 2*math.pi
        elif a_error < -math.pi:
            a_error = a_error + 2*math.pi

        # stall here
        clock.tick(FPS)
        #print(waypoint[0], waypoint[1])       

        for event in pg.event.get():
            if event.type == pg.KEYDOWN:
                key = event.key
                if key == pg.K_q:
                    return

        d_target = get_distance(waypoint, curr_loc)
        #print(d_target, reached[0])
        if d_target < EPSILON_RADIUS and single_ack_entry_flag[0]:
            #print ("WAYPOINT REACHED, d_target = {}".format(d_target))
                #waypoint = points.pop(0)
            reached[0] = True

            a_integral = 0.0
            d_integral = 0.0
            a_prev = 0.0
            d_prev = 0.0
            #continue
            #print("STOPPING")
            #stop()
            stop_cmd = True;
            print("ACK SENT")
            ack_msg = String(data="TRUE")
            ack_publisher.publish(ack_msg)
            single_ack_entry_flag[0] = False
            
            #stop()
            #continue
            
        if stop_cmd:
            print("STOPPING")
            vel = math.sqrt( math.pow(curr_loc[0]-prev_loc[0], 2) + math.pow(curr_loc[1]-prev_loc[1], 2) )
            #print(vel)
            #if vel > 0.1:
                #speed = STOP-0.25
            #else:
            speed = STOP
            stop_cmd = False;
                
            msg = AckermannDriveStamped()
            msg.drive.speed = speed
            msg.drive.steering_angle = 0
            pub.publish(msg)

        if single_ack_entry_flag[0]:
            d_prev = d_target
            d_integral += d_target

            a_prev = a_error
            a_integral += a_error

            if a_error < -EPSILON_ANGLE:
                kLeft = 0
                kRight = 1
            elif a_error > EPSILON_ANGLE:
                kRight = 0
                kLeft = 1
            else:
                kLeft = 0
                kRight = 0

            x_prev = x
            y_prev = y
            
            msg = AckermannDriveStamped()
            # reset speed & direction accordingly.

            speed += DELTA_SPEED_DRIVE * (kUp - kDown)
            direction += -DELTA_DIRECTION * (kRight - kLeft)
            speed = max(min(speed, 2), -2)
            direction = max(min(direction, 0.35), -0.35)
            msg.drive.speed = speed
            msg.drive.steering_angle = direction
            # print(kUp, kDown, kLeft, kRight)
            #print(speed, direction)
            # sys.stdout.flush()
            pub.publish(msg)



def stop():
    #print("STOPPING")
    msg = AckermannDriveStamped()
    msg.drive.speed = STOP
    msg.drive.steering_angle = 0
    pub.publish(msg)


def waypoint_callback(data, args):
    waypoint = args[0]
    single_ack_entry_flag = args[1]   

    if not single_ack_entry_flag[0]:
        single_ack_entry_flag[0] = True
    #print(single_ack_entry_flag[0])
    waypoint[0] = data.x
    waypoint[1] = data.y

    print("WAYPOINT DATA: {}, {}".format(waypoint[0], waypoint[1]))

def main():

    rospy.init_node('waypoint_follower', anonymous=True)   
    try:
        drive()
    finally:
        stop()

main()
#
#communicator()
