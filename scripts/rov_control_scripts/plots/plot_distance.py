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
distance_to_5x5_aruco_marker_approach= []

time_list_dock = []
distance_to_5x5_aruco_marker_dock = []

start_plotting_approach = False
stop_plotting_approach = False
start_plotting_docking = False
stop_plotting_docking = False
display_plot = False

def distance_to_5x5_aruco_marker_and_time_callback(msg):
    global time_list_approach
    global time_list_dock
    global distance_to_5x5_aruco_marker_approach
    global distance_to_5x5_aruco_marker_dock
    

    distance = msg.point.z
    aruco_marker_time_stamp = msg.header.stamp.to_sec()

    if start_plotting_approach and not stop_plotting_approach:
        time_list_approach.append(aruco_marker_time_stamp)
        distance_to_5x5_aruco_marker_approach.append(distance)

    if start_plotting_docking and not stop_plotting_docking:
        time_list_dock.append(aruco_marker_time_stamp)
        distance_to_5x5_aruco_marker_dock.append(distance)

def plot_data():

    fig, ax = plt.subplots(2,1, figsize= (12,8), sharey = False)
    fig.canvas.set_window_title("plot_distance.py")
    fig.suptitle("")
    ax[0].set_title("distance from camera to aruco marker - approach")
    ax[0].set_ylabel("distance (m)")
    ax[0].set_xlabel("simulation time (sec)")
    ax[0].grid()
    ax[0].legend(loc = "upper left")
    ax[1].set_title("distance from camera to aruco marker - docking")
    ax[1].set_ylabel("distance (m)")
    ax[1].set_xlabel("simulation time (sec)")
    ax[1].grid()
    ax[1].legend(loc = "upper left")

    ax[0].plot(time_list_approach, distance_to_5x5_aruco_marker_approach, color = "black", marker = "." , label = "")
    ax[1].plot(time_list_dock, distance_to_5x5_aruco_marker_dock, color = "red", marker = "." , label = "")
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
    stop_plotting_docking = docking_stop.data
    display_plot = docking_stop.data
    if display_plot:
        plot_data()
        rospy.signal_shutdown("Condition met")

if __name__ == "__main__":
    rospy.init_node("distance_from_camera_to_acuco_5x5")
    
    rospy.Subscriber("/rov/aruco_5x5", PointStamped, distance_to_5x5_aruco_marker_and_time_callback)

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
