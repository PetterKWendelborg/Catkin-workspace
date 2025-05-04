#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from matplotlib import pyplot as plt
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

time_list_first_angular_velocity = []
time_list_second_angular_velocity  = []
angular_velocity_z_list_first = []
angular_velocity_z_list_second = []

time_list_first_heading_angle = []
time_list_second_heading_angle  = []
heading_angle_list_second = []
heading_angle_list_first = []


#Det jeg trenger er å vite når den tms heading starter og når den slutter. Som er når den får en verdi til å starte, og sender en ny verdi
start_plotting_outer_rov_heading = False
stop_plotting_outer_rov_heading = False
start_plotting_inner_rov_heading = False
stop_plotting_inner_rov_heading = False
display_plot = False

def heading_angular_velocity_callback(msg):
    global time_list_first_angular_velocity
    global time_list_second_angular_velocity
    global angular_velocity_z_list_first
    global angular_velocity_z_list_second

    imu_time = msg.header.stamp.to_sec()
    angular_velocity_z = msg.angular_velocity.z
    angular_velocity_z_deg = math.degrees(angular_velocity_z)

    if start_plotting_outer_rov_heading and not stop_plotting_outer_rov_heading:
        time_list_first_angular_velocity.append(imu_time)
        angular_velocity_z_list_first.append(angular_velocity_z_deg)

    if start_plotting_inner_rov_heading and not stop_plotting_inner_rov_heading:
        time_list_second_angular_velocity.append(imu_time)
        angular_velocity_z_list_second.append(angular_velocity_z_deg)


def heading_angle_callback(msg):
    global time_list_first_heading_angle
    global time_list_second_heading_angle
    global heading_angle_list_first
    global heading_angle_list_second
    
    heading_angle_time = msg.header.stamp.to_sec()
    heading_angle_value = msg.point.z
    
    # rospy.loginfo(f"haeding time and angle value: {heading_angle_time}, {heading_angle_value} ")
    if start_plotting_outer_rov_heading and not stop_plotting_outer_rov_heading:
        # rospy.loginfo(heading_angle_value)
        time_list_first_heading_angle.append(heading_angle_time)
        heading_angle_list_first.append(heading_angle_value)

    if start_plotting_inner_rov_heading and not stop_plotting_inner_rov_heading:
        time_list_second_heading_angle.append(heading_angle_time)
        heading_angle_list_second.append(heading_angle_value)

def plot_data():

    fig, ax = plt.subplots(2,2, figsize= (14,8), sharey = False)
    fig.canvas.set_window_title("plot_heading_angle_yaw_rov.py")
    fig.suptitle("")

    ax[0,0].set_title("ROV alignment heading - angular velocity in yaw")
    ax[0,0].set_ylabel("angular velocity (deg/sec)")
    ax[0,0].set_xlabel("simulation time (sec)")
    ax[0,0].grid()
    ax[0,0].legend(loc = "upper left")

    ax[1,0].set_title("ROV alignment heading - angular velocity in yaw")
    ax[1,0].set_ylabel("angular velocity (deg/sec)")
    ax[1,0].set_xlabel("simulation time (sec)")
    ax[1,0].grid()
    ax[1,0].legend(loc = "upper left")


    ax[0,1].set_title("ROV correction heading yaw - heading angle")
    ax[0,1].set_ylabel("angle (deg)")
    ax[0,1].set_xlabel("simulation time (sec)")
    ax[0,1].grid()
    ax[0,1].legend(loc = "upper left")

    ax[1,1].set_title("ROV correction heading - heading angle")
    ax[1,1].set_ylabel("angle (deg)")
    ax[1,1].set_xlabel("simulation time (sec)")
    ax[1,1].grid()
    ax[1,1].legend(loc = "upper left")



    ax[0,0].plot(time_list_first_angular_velocity, angular_velocity_z_list_first, color = "black", marker = "." , label = "")
    ax[1,0].plot(time_list_second_angular_velocity, angular_velocity_z_list_second, color = "green", marker = "." , label = "")

    ax[0,1].plot(time_list_first_heading_angle, heading_angle_list_first, color = "black", marker = "." , label = "")
    ax[1,1].plot(time_list_second_heading_angle, heading_angle_list_second, color = "green", marker = "." , label = "")
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
    if display_plot:
        plot_data()
        rospy.signal_shutdown("Condition met")

if __name__ == "__main__":
    rospy.init_node("rov_heading_angle_and_angular_velocity")

    rospy.Subscriber("/rov/imu", Imu, heading_angular_velocity_callback)
    rospy.Subscriber("/rov/tms_center_angle", PointStamped, heading_angle_callback)

    rospy.Subscriber("/tms/heading_done", Bool, start_outer_heading_call)
    rospy.Subscriber("/rov/heading_done", Bool, stop_outer_heading_call)
    rospy.Subscriber("/tms/inner_heading_done", Bool, start_inner_heading_call)
    rospy.Subscriber("/rov/inner_heading_done", Bool, stop_inner_heading_call)

    rospy.Subscriber("/rov/docking_done", Bool, plot_call)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()