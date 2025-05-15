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

start_plotting_approach = False
stop_plotting_approach = False
display_plot = False

time_list_velocity_x_approach = []
linear_velocity_x_list_approach = []


time_list_velocity_x_terminal_docking = []
linear_velocity_x_list_terminal_docking = []

last_time = None
linear_velocity_x = 0.0

time_display_plot = None

def FLS_distance_to_TMS_and_time_callback(msg):
    global time_list_approach
    global fls_distance_to_tms_approach

    distance = msg.point.x
    aruco_marker_time_stamp = msg.header.stamp.to_sec()

    if start_plotting_approach and not stop_plotting_approach:
        time_list_approach.append(aruco_marker_time_stamp)
        fls_distance_to_tms_approach.append(distance)

def time_and_linear_acceleration_callback(msg):
    global time_list_velocity_x_approach
    global linear_velocity_x_list_approach
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

def plot_data():

    fig, ax = plt.subplots(2, figsize= (14,8), sharey = False)
    fig.canvas.set_window_title("plot_approach_2d.py")
    fig.suptitle("")

    ax[0].set_title("distance from FLS to TMS - approach")
    ax[0].set_ylabel("distance (m)")
    ax[0].set_xlabel("simulation time (sec)")
    ax[0].grid()
    ax[0].legend(loc = "upper left")

    ax[1].set_title("ROV linear velocity in x axis - approach")
    ax[1].set_ylabel("linear velocity (m/s)")
    ax[1].set_xlabel("simulation time (sec)")
    ax[1].grid()
    ax[1].legend(loc = "upper left")

    ax[0].plot(time_list_approach, fls_distance_to_tms_approach, color = "red", marker = "." , label = "")

    ax[1].plot(time_list_velocity_x_approach, linear_velocity_x_list_approach, color = "red", marker = "." , label = "")
    fig.tight_layout()

    plt.show()

def start_approach_call(approach_start):
    global start_plotting_approach
    start_plotting_approach = approach_start.data

def stop_approach_call(approach_stop):
    global stop_plotting_approach
    global display_plot
    stop_plotting_approach = approach_stop.data
    display_plot = approach_stop.data

    if display_plot:
        plot_data()
        rospy.signal_shutdown("Condition met")

    # time_display_plot_duration = 10
    
    # display_plot = approach_stop.data
    # now = rospy.get_time()

    # if display_plot:
    #     if time_display_plot == None:
    #         time_display_plot = now

    #     elif now - time_display_plot >= time_display_plot_duration:
    #         stop_plotting_approach = approach_stop.data
    #         plot_data()
    #         rospy.signal_shutdown("Condition met")

if __name__ == "__main__":
    rospy.init_node("distance_and_linear_velocit_x")
    
    rospy.Subscriber("/rov/imu", Imu, time_and_linear_acceleration_callback)
    rospy.Subscriber("/rov/tms_center", PointStamped, FLS_distance_to_TMS_and_time_callback)

    rospy.Subscriber("/rov/heading_done", Bool, start_approach_call)
    rospy.Subscriber("/rov/approach_done", Bool, stop_approach_call)

    # rospy.Subscriber("/rov/heading_done", Bool, stop_docking_call)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()
