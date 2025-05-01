#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from matplotlib import pyplot as plt
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

time_list_first = []
distance_to_5x5_aruco_marker_first= []

time_list_second = []
distance_to_5x5_aruco_marker_second = []

start_plotting_outer_rov_heading = False
stop_plotting_outer_rov_heading = False
start_plotting_inner_rov_heading = False
stop_plotting_inner_rov_heading = False
display_plot = False

def time_and_angular_velocity_callback(msg):
    global time_list_first
    global time_list_second

    imu_time = msg.header.stamp.to_sec()
    angular_velocity_z = msg.angular_velocity.z

    if not stop_plotting_outer_rov_heading:
        time_list_first.append(imu_time)

    if start_plotting_inner_rov_heading and not stop_plotting_inner_rov_heading:
        time_list_second.append(imu_time)

    if display_plot:
        plot_data()
        rospy.signal_shutdown("Condition met")

def distance_to_5x5_aruco_marker_callback(msg):

    if not stop_plotting_outer_rov_heading:

        global distance_to_5x5_aruco_marker_first
        global distance_to_5x5_aruco_marker_second
        time_list_first.append(imu_time)
        angular_velocity_z_list_first.append(angular_velocity_z)

    if start_plotting_inner_rov_heading and not stop_plotting_inner_rov_heading:
        time_list_second.append(imu_time)
        angular_velocity_z_list_second.append(angular_velocity_z)


def plot_data():

    fig, ax = plt.subplots(2,1, figsize= (14,10), sharey = False)
    fig.suptitle("")
    ax[0].set_title("First itteration of ROV heading - angular velocity in z axis")
    ax[0].set_ylabel("angular velocity (rad/sec)")
    ax[0].set_xlabel("simulation time (sec)")
    ax[0].grid()
    ax[0].legend(loc = "upper left")
    ax[1].set_title("Second itteration of ROV heading - angular velocity in z axis")
    ax[1].set_ylabel("angular velocity (rad/sec)")
    ax[1].set_xlabel("simulation time (sec)")
    ax[1].grid()
    ax[1].legend(loc = "upper left")

    ax[0].plot(time_list_first, distance_to_5x5_aruco_marker_first, color = "black", marker = "." , label = "")
    ax[1].plot(time_list_second, distance_to_5x5_aruco_marker_second, color = "green", marker = "." , label = "")
    fig.tight_layout()

    plt.show()

def start_outer_heading_call(outer_rov_heading_start):
    global start_plotting_outer_rov_heading
    start_plotting_outer_rov_heading = outer_rov_heading_start.data

def stop_outer_heading_call(outer_rov_heading_stop):
    global stop_plotting_outer_rov_heading
    stop_plotting_outer_rov_heading = outer_rov_heading_stop.data

def start_inner_heading_call(inner_rov_heading_start):
    global start_plotting_inner_rov_heading
    start_plotting_inner_rov_heading = inner_rov_heading_start.data

def stop_inner_heading_call(inner_rov_heading_stop):
    global stop_plotting_inner_rov_heading
    stop_plotting_inner_rov_heading = inner_rov_heading_stop.data

def plot_call(docking_done):
    global display_plot
    display_plot = docking_done.data

if __name__ == "__main__":
    rospy.init_node("rov_angular_velocity_yaw")

    rospy.Subscriber("/rov/imu", Imu, time_and_angular_velocity_callback)
    rospy.Subscriber("/rov/aruco_5x5", Point, time_and_angular_velocity_callback)
    rospy.Subscriber("/tms/heading_done", Bool, start_outer_heading_call)
    rospy.Subscriber("/rov/heading_done", Bool, stop_outer_heading_call)
    rospy.Subscriber("/tms/inner_heading_done", Bool, start_inner_heading_call)
    rospy.Subscriber("/rov/inner_heading_done", Bool, stop_inner_heading_call)

    rospy.Subscriber("/rov/docking_done", Bool, plot_call)

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