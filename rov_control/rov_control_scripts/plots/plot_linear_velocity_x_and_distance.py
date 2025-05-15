#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from matplotlib import pyplot as plt
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PointStamped

import time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

time_list_approach = []
fls_distance_to_tms_approach= []

time_list_dock = []
distance_to_5x5_aruco_marker_dock = []

start_plotting_approach = False
stop_plotting_approach = False
start_plotting_docking = False
stop_plotting_docking = False
display_plot = False

time_list_velocity_x_approach = []
linear_velocity_x_list_approach = []
# linear_velocity_x_approach = 0.0
# last_time_approach = None

time_list_velocity_x_terminal_docking = []
linear_velocity_x_list_terminal_docking = []
# linear_velocity_x_terminal_docking = 0.0
# last_time_terminal_docking = None
last_time = None
linear_velocity_x = 0.0

time_display_plot = None

# terminal docking protocoll distance and time using camera
def camera_distance_to_5x5_aruco_marker_and_time_callback(msg):
    # global time_list_approach
    global time_list_dock
    # global distance_to_5x5_aruco_marker_approach
    global distance_to_5x5_aruco_marker_dock
    

    distance = msg.point.z
    aruco_marker_time_stamp = msg.header.stamp.to_sec()

    # if start_plotting_approach and not stop_plotting_approach:
    #     time_list_approach.append(aruco_marker_time_stamp)
    #     distance_to_5x5_aruco_marker_approach.append(distance)

    if start_plotting_docking and not stop_plotting_docking:
        time_list_dock.append(aruco_marker_time_stamp)
        distance_to_5x5_aruco_marker_dock.append(distance)


# approach protocoll distance and time using FLS
def FLS_distance_to_TMS_and_time_callback(msg):
    global time_list_approach
    # global time_list_dock
    global fls_distance_to_tms_approach
    # global distance_to_5x5_aruco_marker_dock
    

    distance = msg.point.z
    aruco_marker_time_stamp = msg.header.stamp.to_sec()

    if start_plotting_approach and not stop_plotting_approach:
        time_list_approach.append(aruco_marker_time_stamp)
        fls_distance_to_tms_approach.append(distance)

    # if start_plotting_docking and not stop_plotting_docking:
    #     time_list_dock.append(aruco_marker_time_stamp)
    #     distance_to_5x5_aruco_marker_dock.append(distance)

# approach/terminal docking protocoll linear_velocity_x
def time_and_linear_acceleration_callback(msg):
    global time_list_velocity_x_approach
    global time_list_velocity_x_terminal_docking
    global linear_velocity_x_list_approach
    global linear_velocity_x_list_terminal_docking
    global last_time
    global linear_velocity_x 

    current_time = msg.header.stamp.to_sec()
    linear_acceleration_x = msg.linear_acceleration.x
    
    if last_time is None:
        last_time = current_time
        return
    
    delta_time = current_time - last_time
    last_time = current_time
    linear_velocity_x += linear_acceleration_x * delta_time
    # rospy.loginfo(f"\ndelta_time: {delta_time} \ncurrent_time: {current_time}\nlast_time: {last_time} \nlinear_velocity_x:{linear_velocity_x} \nlinear_acceleration_x:{linear_acceleration_x}\n")
    
    if start_plotting_approach and not stop_plotting_approach:
        time_list_velocity_x_approach.append(current_time)
        linear_velocity_x_list_approach.append(linear_velocity_x)

    if start_plotting_docking and not stop_plotting_docking:
        time_list_velocity_x_terminal_docking.append(current_time)
        linear_velocity_x_list_terminal_docking.append(linear_velocity_x)

def plot_data():

    fig, ax = plt.subplots(2,2, figsize= (14,8), sharey = False)
    fig.canvas.set_window_title("plot_linear_velocity_x_and_distance.py")
    fig.suptitle("")

    ax[0,0].set_title("distance from FLS to TMS - approach")
    ax[0,0].set_ylabel("distance (m)")
    ax[0,0].set_xlabel("simulation time (sec)")
    ax[0,0].grid()
    ax[0,0].legend(loc = "upper left")

    ax[1,0].set_title("distance from camera to aruco marker - terminal docking")
    ax[1,0].set_ylabel("distance (m)")
    ax[1,0].set_xlabel("simulation time (sec)")
    ax[1,0].grid()
    ax[1,0].legend(loc = "upper left")


    ax[0,1].set_title("ROV linear velocity in x axis - approach")
    ax[0,1].set_ylabel("linear velocity (m/s)")
    ax[0,1].set_xlabel("simulation time (sec)")
    ax[0,1].grid()
    ax[0,1].legend(loc = "upper left")

    ax[1,1].set_title("ROV linear velocity in x axis - terminal docking")
    ax[1,1].set_ylabel("linear velocity (m/s)")
    ax[1,1].set_xlabel("simulation time (sec)")
    ax[1,1].grid()
    ax[1,1].legend(loc = "upper left")

    ax[0,0].plot(time_list_approach, fls_distance_to_tms_approach, color = "red", marker = "." , label = "")
    ax[1,0].plot(time_list_dock, distance_to_5x5_aruco_marker_dock, color = "black", marker = "." , label = "")

    ax[0,1].plot(time_list_velocity_x_approach, linear_velocity_x_list_approach, color = "red", marker = "." , label = "")
    ax[1,1].plot(time_list_velocity_x_terminal_docking, linear_velocity_x_list_terminal_docking, color = "black", marker = "." , label = "")
    fig.tight_layout()

    plt.show()

def start_approach_call(approach_start):
    global start_plotting_approach
    start_plotting_approach = approach_start.data

def stop_approach_call(approach_stop):
    global stop_plotting_approach
    stop_plotting_approach = approach_stop.data

def start_docking_call(docking_start):
    global start_plotting_docking
    start_plotting_docking = docking_start.data

def stop_docking_call(docking_stop):
    global stop_plotting_docking
    global display_plot
    global time_display_plot

    time_display_plot_duration = 10

    
    display_plot = docking_stop.data
    now = rospy.get_time()

    if display_plot:
        if time_display_plot == None:
            time_display_plot = now

        elif now - time_display_plot >= time_display_plot_duration:
            stop_plotting_docking = docking_stop.data
            plot_data()
            rospy.signal_shutdown("Condition met")

if __name__ == "__main__":
    rospy.init_node("distance_and_linear_velocit_x")
    
    rospy.Subscriber("/rov/aruco_5x5", PointStamped, camera_distance_to_5x5_aruco_marker_and_time_callback)
    rospy.Subscriber("/rov/imu", Imu, time_and_linear_acceleration_callback)
    rospy.Subscriber("/rov/tms_center", PointStamped, FLS_distance_to_TMS_and_time_callback)

    rospy.Subscriber("/rov/far_alignment_done", Bool, start_approach_call)
    rospy.Subscriber("/rov/approach_done", Bool, stop_approach_call)
    rospy.Subscriber("/rov/alignment_done", Bool, start_docking_call)
    rospy.Subscriber("/rov/docking_done", Bool, stop_docking_call)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()
