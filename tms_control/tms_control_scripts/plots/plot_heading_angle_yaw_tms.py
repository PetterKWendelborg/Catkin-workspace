#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from matplotlib import pyplot as plt
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

alignment_heading_time_list_angular_velocity = []
correction_heading_time_list_angular_velocity = []
alignment_heading_angular_velocity_yaw_list = []
correction_heading_angular_velocity_yaw_list = []

alignment_heading_time_list_heading_angle = []
correction_heading_time_list_heading_angle = []
alignment_heading_angle_yaw_list= []
correction_heading_angle_yaw_list = []

start_plotting_outer_tms_heading = False
stop_plotting_outer_tms_heading = False
start_plotting_inner_tms_heading = False
stop_plotting_inner_tms_heading = False
display_plot = False

def heading_angular_velocity_callback(msg):
    global alignment_heading_time_list_angular_velocity
    global correction_heading_time_list_angular_velocity
    global alignment_heading_angular_velocity_yaw_list
    global correction_heading_angular_velocity_yaw_list

    imu_time = msg.header.stamp.to_sec()
    angular_velocity_z = msg.angular_velocity.z
    angular_velocity_z_deg = math.degrees(angular_velocity_z)

    if not stop_plotting_outer_tms_heading:
        alignment_heading_time_list_angular_velocity.append(imu_time)
        alignment_heading_angular_velocity_yaw_list.append(angular_velocity_z_deg)

    if start_plotting_inner_tms_heading and not stop_plotting_inner_tms_heading:
        correction_heading_time_list_angular_velocity.append(imu_time)
        correction_heading_angular_velocity_yaw_list.append(angular_velocity_z_deg)

def heading_angle_callback(msg):
    global alignment_heading_time_list_heading_angle
    global correction_heading_time_list_heading_angle
    global alignment_heading_angle_yaw_list
    global correction_heading_angle_yaw_list
    
    heading_angle_time = msg.header.stamp.to_sec()
    heading_angle_value = msg.point.z
    
    if not stop_plotting_outer_tms_heading:
        alignment_heading_time_list_heading_angle.append(heading_angle_time)
        alignment_heading_angle_yaw_list.append(heading_angle_value)

    if start_plotting_inner_tms_heading and not stop_plotting_inner_tms_heading:
        correction_heading_time_list_heading_angle.append(heading_angle_time)
        correction_heading_angle_yaw_list.append(heading_angle_value)

def plot_data():

    fig, ax = plt.subplots(2,2, figsize= (14,8), sharey = False)
    fig.canvas.set_window_title("plot_heading_angle_yaw_tms.py")
    fig.suptitle("")

    ax[0,0].set_title("TMS rough heading - angular velocity yaw")
    ax[0,0].set_ylabel("angular velocity (deg/sec)")
    ax[0,0].set_xlabel("simulation time (sec)")
    ax[0,0].grid()
    ax[0,0].legend(loc = "upper left")

    ax[1,0].set_title("TMS fine heading - angular velocity yaw")
    ax[1,0].set_ylabel("angular velocity (deg/sec)")
    ax[1,0].set_xlabel("simulation time (sec)")
    ax[1,0].grid()
    ax[1,0].legend(loc = "upper left")

    ax[0,1].set_title("TMS rough heading yaw - heading angle")
    ax[0,1].set_ylabel("angle (deg)")
    ax[0,1].set_xlabel("simulation time (sec)")
    ax[0,1].grid()
    ax[0,1].legend(loc = "upper left")

    ax[1,1].set_title("TMS fine heading - heading angle")
    ax[1,1].set_ylabel("angle (deg)")
    ax[1,1].set_xlabel("simulation time (sec)")
    ax[1,1].grid()
    ax[1,1].legend(loc = "upper left")

    ax[0,0].plot(alignment_heading_time_list_angular_velocity, alignment_heading_angular_velocity_yaw_list, color = "blue", marker = "." , label = "")
    ax[1,0].plot(correction_heading_time_list_angular_velocity, correction_heading_angular_velocity_yaw_list, color = "red", marker = "." , label = "")

    ax[0,1].plot(alignment_heading_time_list_heading_angle, alignment_heading_angle_yaw_list, color = "blue", marker = "." , label = "")
    ax[1,1].plot(correction_heading_time_list_heading_angle, correction_heading_angle_yaw_list, color = "red", marker = "." , label = "")
    fig.tight_layout()

    plt.show()

def stop_outer_heading_call(outer_tms_heading_stop):
    global stop_plotting_outer_tms_heading
    stop_plotting_outer_tms_heading = outer_tms_heading_stop.data
        
def start_inner_heading_call(inner_tms_heading_start):
    global start_plotting_inner_tms_heading
    start_plotting_inner_tms_heading = inner_tms_heading_start.data

def stop_inner_heading_call(inner_tms_heading_stop):
    global stop_plotting_inner_tms_heading
    stop_plotting_inner_tms_heading = inner_tms_heading_stop.data

def plot_call(docking_done):
    global display_plot
    display_plot = docking_done.data
    if display_plot:
        plot_data()
        rospy.signal_shutdown("Condition met")

if __name__ == "__main__":
    rospy.init_node("tms_heading_angle_and_angular_velocity")

    rospy.Subscriber("/tms/imu", Imu, heading_angular_velocity_callback)
    rospy.Subscriber("/tms/rov_center_angle", PointStamped, heading_angle_callback)

    rospy.Subscriber("/tms/heading_done", Bool, stop_outer_heading_call)
    rospy.Subscriber("/rov/approach_done", Bool, start_inner_heading_call)
    rospy.Subscriber("/tms/inner_heading_done", Bool, stop_inner_heading_call)

    rospy.Subscriber("/rov/docking_done", Bool, plot_call)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()