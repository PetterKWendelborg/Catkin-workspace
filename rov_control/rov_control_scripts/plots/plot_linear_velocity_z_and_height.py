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


start_plotting_alignment_rough = False
stop_plotting_alignment_rough = False
start_plotting_alignment_fine = False
stop_plotting_alignment_fine = False
display_plot = False

time_list_height_alignment_rough = []
fls_height_to_tms_alignment_rough= []

time_list_height_alignment_fine = []
camera_height_to_tms_alignment_fine = []

time_list_velocity_z_rough = []
linear_velocity_z_list_rough = []

time_list_velocity_z_fine = []
linear_velocity_z_list_fine= []

last_time = None
linear_velocity_z = 0.0

time_display_plot = None

def fls_height_to_TMS_and_time_callback(msg):
    global time_list_height_alignment_rough
    global fls_height_to_tms_alignment_rough

    height = msg.point.y
    aruco_marker_time_stamp = msg.header.stamp.to_sec()

    if start_plotting_alignment_rough and not stop_plotting_alignment_rough:
        time_list_height_alignment_rough.append(aruco_marker_time_stamp)
        fls_height_to_tms_alignment_rough.append(height)

def camera_height_to_4x4_aruco_marker_and_time_callback(msg):
    global time_list_height_alignment_fine
    global camera_height_to_tms_alignment_fine
    
    height = msg.point.y
    aruco_marker_time_stamp = msg.header.stamp.to_sec()

    if start_plotting_alignment_fine and not stop_plotting_alignment_fine:
        time_list_height_alignment_fine.append(aruco_marker_time_stamp)
        camera_height_to_tms_alignment_fine.append(height)



def time_and_linear_acceleration_callback(msg):
    global time_list_velocity_z_rough
    global linear_velocity_z_list_rough
    global time_list_velocity_z_fine
    global linear_velocity_z_list_fine
    global last_time
    global linear_velocity_z

    current_time = msg.header.stamp.to_sec()
    linear_acceleration_z_gravity = msg.linear_acceleration.z

    #removing the accelration from gravity
    linear_acceleration_z = linear_acceleration_z_gravity - 9.8 #gotten from the /rov/imu topic
    
    if last_time is None:
        last_time = current_time
        return
    
    delta_time = current_time - last_time
    last_time = current_time
    linear_velocity_z += linear_acceleration_z * delta_time
    # rospy.loginfo(f"\ndelta_time: {delta_time} \ncurrent_time: {current_time}\nlast_time: {last_time} \nlinear_velocity_x:{linear_velocity_z} \nlinear_acceleration_x:{linear_acceleration_z}\n")
    
    if start_plotting_alignment_rough and not stop_plotting_alignment_rough:
        time_list_velocity_z_rough.append(current_time)
        linear_velocity_z_list_rough.append(linear_velocity_z)

    if start_plotting_alignment_fine and not stop_plotting_alignment_fine:
        time_list_velocity_z_fine.append(current_time)
        linear_velocity_z_list_fine.append(linear_velocity_z)

def plot_data():

    fig, ax = plt.subplots(2,2, figsize= (14,8), sharey = False)
    fig.canvas.set_window_title("plot_linear_velocity_z_and_height.py")
    fig.suptitle("")

    ax[0,0].set_title("Height from FLS to TMS - height alignment rough")
    ax[0,0].set_ylabel("height (m)")
    ax[0,0].set_xlabel("simulation time (sec)")
    ax[0,0].grid()
    ax[0,0].legend(loc = "upper left")

    ax[1,0].set_title("Height from camera to aruco marker - height alignment fine")
    ax[1,0].set_ylabel("height (m)")
    ax[1,0].set_xlabel("simulation time (sec)")
    ax[1,0].grid()
    ax[1,0].legend(loc = "upper left")


    ax[0,1].set_title("ROV linear velocity in z axis - height alignment rough")
    ax[0,1].set_ylabel("linear velocity (m/s)")
    ax[0,1].set_xlabel("simulation time (sec)")
    ax[0,1].grid()
    ax[0,1].legend(loc = "upper left")

    ax[1,1].set_title("ROV linear velocity in z axis - height alignment fine")
    ax[1,1].set_ylabel("linear velocity (m/s)")
    ax[1,1].set_xlabel("simulation time (sec)")
    ax[1,1].grid()
    ax[1,1].legend(loc = "upper left")

    ax[0,0].plot(time_list_height_alignment_rough, fls_height_to_tms_alignment_rough, color = "red", marker = "." , label = "")
    ax[1,0].plot(time_list_height_alignment_fine, camera_height_to_tms_alignment_fine, color = "black", marker = "." , label = "")

    ax[0,1].plot(time_list_velocity_z_rough, linear_velocity_z_list_rough, color = "red", marker = "." , label = "")
    ax[1,1].plot(time_list_velocity_z_fine, linear_velocity_z_list_fine, color = "black", marker = "." , label = "")
    fig.tight_layout()

    plt.show()

def start_height_rough_call(height_alignment_rough_start):
    global start_plotting_alignment_rough
    start_plotting_alignment_rough = height_alignment_rough_start.data

def stop_height_rough_call(height_alignment_rough_stop):
    global stop_plotting_alignment_rough
    stop_plotting_alignment_rough = height_alignment_rough_stop.data

def start_height_fine_call(height_alignment_fine_start):
    global start_plotting_alignment_fine
    start_plotting_alignment_fine = height_alignment_fine_start.data

def stop_height_fine_call(height_alignment_fine_stop):
    global stop_plotting_alignment_fine
    stop_plotting_alignment_fine = height_alignment_fine_stop.data

def plot_call(docking_done):
    global display_plot
    global time_display_plot

    time_display_plot_duration = 8
    
    now = rospy.get_time()
    display_plot = docking_done.data

    if display_plot:
        plot_data()
        rospy.signal_shutdown("Condition met")

        # if time_display_plot == None:
        #     time_display_plot = now

        # elif now - time_display_plot >= time_display_plot_duration:
        #     stop_plotting_alignment_fine = height_alignment_fine_stop.data
        #     plot_data()
        #     rospy.signal_shutdown("Condition met")

if __name__ == "__main__":
    rospy.init_node("height_and_linear_velocit_z")
    
    rospy.Subscriber("/rov/aruco_4x4", PointStamped, camera_height_to_4x4_aruco_marker_and_time_callback)
    rospy.Subscriber("/rov/imu", Imu, time_and_linear_acceleration_callback)
    rospy.Subscriber("/rov/tms_center", PointStamped, fls_height_to_TMS_and_time_callback)

    rospy.Subscriber("/rov/heading_done", Bool, start_height_rough_call)
    rospy.Subscriber("/rov/far_alignment_done", Bool, stop_height_rough_call)
    rospy.Subscriber("/rov/inner_heading_done", Bool, start_height_fine_call)
    rospy.Subscriber("/rov/alignment_done", Bool, stop_height_fine_call)


    rospy.Subscriber("/rov/docking_done", Bool, plot_call)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()
