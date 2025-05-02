#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from matplotlib import pyplot as plt
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import time

time_list_first = []
distance_to_5x5_aruco_marker_first= []

time_list_second = []
distance_to_5x5_aruco_marker_second = []

start_plotting_approach = False
stop_plotting_approach = False
start_plotting_docking = False
stop_plotting_docking = False
display_plot = False

aruco_update_time = 0


# def time_and_angular_velocity_callback(msg):
#     global time_list_first
#     global time_list_second
    
#     # imu_time = msg.header.stamp.to_sec()
#     imu_time = msg.Clock()
#     time_list_first.append(imu_time)


def time_and_angular_velocity_callback(msg):
    global time_list_first
    global time_list_second
    
    imu_time = msg.header.stamp.to_sec()
    time_list_first.append(imu_time)
    # if start_plotting_approach and not stop_plotting_approach:
    #     time_list_first.append(imu_time)

    # if start_plotting_docking and not stop_plotting_docking:
    #     time_list_second.append(imu_time)

    # if stop_plotting_approach:
    #     plot_data()
    #     rospy.signal_shutdown("Condition met")

def distance_to_5x5_aruco_marker_callback(aruco_msg):
    global distance_to_5x5_aruco_marker_first
    global distance_to_5x5_aruco_marker_second
    global aruco_update_time


    distance = aruco_msg.z
    distance_to_5x5_aruco_marker_first.append(distance)

    # if start_plotting_approach and not stop_plotting_approach:
    #     distance_to_5x5_aruco_marker_first.append(distance)

    # if start_plotting_docking and not stop_plotting_docking:
    #     distance_to_5x5_aruco_marker_second.append(distance)


def plot_data():

    fig, ax = plt.subplots(2,1, figsize= (14,10), sharey = False)
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

    ax[0].plot(time_list_first, distance_to_5x5_aruco_marker_first, color = "black", marker = "." , label = "")
    ax[1].plot(time_list_second, distance_to_5x5_aruco_marker_second, color = "red", marker = "." , label = "")
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

# def plot_call(docking_done):
#     global display_plot
#     display_plot = docking_done.data
#     if display_plot:
#         plot_data()
#         rospy.signal_shutdown("Condition met")

# def plot_call(docking_done):
#     global display_plot
#     display_plot = docking_done.data

if __name__ == "__main__":
    rospy.init_node("distance_from_camera_to_acuco_5x5")

    rospy.Subscriber("/rov/imu", Clock, time_and_angular_velocity_callback)
    # rospy.Subscriber("/rov/imu", Imu, time_and_angular_velocity_callback)
    
    rospy.Subscriber("/rov/aruco_5x5", Point, distance_to_5x5_aruco_marker_callback)

    rospy.Subscriber("/rov/heading_done", Bool, start_approach_call)
    rospy.Subscriber("/rov/approach_done", Bool, stop_approach_call)
    rospy.Subscriber("/rov/alignment_done", Bool, start_docking_call)
    rospy.Subscriber("/rov/docking_done", Bool, stop_docking_call)
    # rospy.Subscriber("/rov/docking_done", Bool, plot_call)

    # rospy.Subscriber("/rov/docking_done", Bool, plot_call)

    # rospy.spin()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()





# time_list = []
# angular_velocity_z_list = []

# def time_and_angular_velocity_callback(msg):
#     global time_list
#     global angular_velocity_z_list 

#     imu_time = msg.header.stamp.to_sec()
#     angular_velocity_z = msg.angular_velocity.z
    
#     # rospy.loginfo(f"runtime and torque: {imu_time: .1f}, {angular_velocity_z: .5f} ")
#     time_list.append(imu_time)
#     angular_velocity_z_list.append(angular_velocity_z)

# def plot_data():

#     plt.figure(figsize= (12,8))
#     plt.plot(time_list, angular_velocity_z_list, color = "black", marker = "." , label = "")
#     plt.title("ROV angular velocity in z axis")
#     plt.xlabel("simulation time (sec)")
#     plt.ylabel("angular velocity (rad/sec)")
#     plt.grid()
#     plt.legend(loc = "upper left")
#     plt.show()

# if __name__ == "__main__":
#     rospy.init_node("ROV_angular_velocity_z")

#     rospy.Subscriber("/rov/imu", Imu, time_and_angular_velocity_callback)

#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         plot_data()