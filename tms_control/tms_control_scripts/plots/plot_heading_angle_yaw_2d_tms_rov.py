#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from matplotlib import pyplot as plt
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

tms_heading_time_list_angular_velocity = []
tms_heading_angular_velocity_yaw_list = []

tms_heading_time_list_heading_angle = []
tms_heading_angle_yaw_list= []

rov_heading_time_list_angular_velocity = []
rov_heading_angular_velocity_yaw_list = []

rov_heading_time_list_heading_angle = []
rov_heading_angle_yaw_list = []

start_plotting_tms_heading = False
stop_plotting_tms_heading = False

start_plotting_rov_heading = False
stop_plotting_rov_heading = False
display_plot = False


def tms_heading_angular_velocity_callback(msg):
    global tms_heading_time_list_angular_velocity
    global tms_heading_angular_velocity_yaw_list

    imu_time = msg.header.stamp.to_sec()
    angular_velocity_yaw  = msg.angular_velocity.z
    angular_velocity_z_deg = math.degrees(angular_velocity_yaw)

    if not stop_plotting_tms_heading:
        tms_heading_time_list_angular_velocity.append(imu_time)
        tms_heading_angular_velocity_yaw_list.append(angular_velocity_z_deg)


def rov_heading_angular_velocity_callback(msg):
    global rov_heading_time_list_angular_velocity
    global rov_heading_angular_velocity_yaw_list

    imu_time = msg.header.stamp.to_sec()
    angular_velocity_z = msg.angular_velocity.z
    angular_velocity_z_deg = math.degrees(angular_velocity_z)

    if stop_plotting_tms_heading and not stop_plotting_rov_heading:
        rov_heading_time_list_angular_velocity.append(imu_time)
        rov_heading_angular_velocity_yaw_list.append(angular_velocity_z_deg)

def tms_heading_angle_callback(msg):
    global tms_heading_time_list_heading_angle
    global tms_heading_angle_yaw_list
    
    heading_angle_time = msg.header.stamp.to_sec()
    heading_angle_value = msg.point.z
    
    if not stop_plotting_tms_heading:
        tms_heading_time_list_heading_angle.append(heading_angle_time)
        tms_heading_angle_yaw_list.append(heading_angle_value)

def rov_heading_angle_callback(msg):
    global rov_heading_time_list_heading_angle
    global rov_heading_angle_yaw_list

    heading_angle_time = msg.header.stamp.to_sec()
    heading_angle_value = msg.point.z
    
    if stop_plotting_tms_heading and not stop_plotting_rov_heading:
        rov_heading_time_list_heading_angle.append(heading_angle_time)
        rov_heading_angle_yaw_list.append(heading_angle_value)

def plot_data():

    fig, ax = plt.subplots(2,2, figsize= (14,8), sharey = False)
    fig.canvas.set_window_title("plot_heading_angle_yaw_2d_tms_rov.py")
    fig.suptitle("")

    ax[0,0].set_title("TMS heading - angular velocity yaw")
    ax[0,0].set_ylabel("angular velocity (deg/sec)")
    ax[0,0].set_xlabel("simulation time (sec)")
    ax[0,0].grid()
    ax[0,0].legend(loc = "upper left")

    ax[1,0].set_title("ROV heading - angular velocity yaw")
    ax[1,0].set_ylabel("angular velocity (deg/sec)")
    ax[1,0].set_xlabel("simulation time (sec)")
    ax[1,0].grid()
    ax[1,0].legend(loc = "upper left")

    ax[0,1].set_title("ROV heading yaw - heading angle")
    ax[0,1].set_ylabel("angle (deg)")
    ax[0,1].set_xlabel("simulation time (sec)")
    ax[0,1].grid()
    ax[0,1].legend(loc = "upper left")

    ax[1,1].set_title("ROV heading - heading angle")
    ax[1,1].set_ylabel("angle (deg)")
    ax[1,1].set_xlabel("simulation time (sec)")
    ax[1,1].grid()
    ax[1,1].legend(loc = "upper left")

    ax[0,0].plot(tms_heading_time_list_angular_velocity, tms_heading_angular_velocity_yaw_list, color = "blue", marker = "." , label = "")
    ax[0,1].plot(tms_heading_time_list_heading_angle, tms_heading_angle_yaw_list, color = "blue", marker = "." , label = "")

    ax[1,0].plot(rov_heading_time_list_angular_velocity, rov_heading_angular_velocity_yaw_list, color = "black", marker = "." , label = "")
    ax[1,1].plot(rov_heading_time_list_heading_angle, rov_heading_angle_yaw_list, color = "green", marker = "." , label = "")
    fig.tight_layout()

    plt.show()

def stop_tms_heading_call(tms_heading_stop): #also starts the ROV heading after the TMS heading is done
    global stop_plotting_tms_heading
    stop_plotting_tms_heading = tms_heading_stop.data
        
# def start_rov_heading_call(rov_heading_start):
#     global start_plotting_rov_heading
#     start_plotting_inner_tms_heading = rov_heading_start.data

def stop_rov_heading_call(rov_heading_stop):
    global stop_plotting_rov_heading
    stop_plotting_rov_heading = rov_heading_stop.data

def plot_call(approach_done):
    global display_plot
    display_plot = approach_done.data
    if display_plot:
        plot_data()
        rospy.signal_shutdown("Condition met")

if __name__ == "__main__":
    rospy.init_node("tms_heading_angle_and_angular_velocity")

    rospy.Subscriber("/tms/imu", Imu, tms_heading_angular_velocity_callback)
    rospy.Subscriber("/tms/rov_center_angle", PointStamped, tms_heading_angle_callback)

    rospy.Subscriber("/rov/imu", Imu, rov_heading_angular_velocity_callback)
    rospy.Subscriber("/rov/rov_center_angle", PointStamped, rov_heading_angle_callback)

    rospy.Subscriber("/tms/heading_done", Bool, stop_tms_heading_call)
    # rospy.Subscriber("/tms/heading_done", Bool, start_rov_heading_call)
    rospy.Subscriber("/rov/heading_done", Bool, stop_rov_heading_call)

    rospy.Subscriber("/rov/aproach_done", Bool, plot_call)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()