#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from matplotlib import pyplot as plt
from std_msgs.msg import Bool

time_list_first = []
angular_velocity_z_list_first = []

time_list_second = []
angular_velocity_z_list_second = []

#Det blir nok motsatt logikk, dette er fordi den boolean verdien som hentes fra de forskjellige prosodyrene
#sender en TRUE statement som skal stoppe hentingen av verdier til listene
#Faktisk kanskje ikke, jeg har 2 valg. Hver script i docking prossodyren 
#"henter" en boolean verdi for å starte, 
#og "sender" en boolean verdi for å starte en ny prosess. Deretter terminater den seg selv.

#Det jeg trenger er å vite når den tms heading starter og når den slutter. Som er når den får en verdi til å starte, og sender en ny verdi
stop_plotting_outer_tms_heading = False
start_plotting_inner_tms_heading = False
stop_plotting_inner_tms_heading = False

def time_and_angular_velocity_callback(msg):
    global time_list_first
    global angular_velocity_z_list_first
    global time_list_second
    global angular_velocity_z_list_second
    outer_start_stop_initiator = 2
    inner_start_stop_initiator = 2
    start_stop_limit = 4

    imu_time = msg.header.stamp.to_sec()
    angular_velocity_z = msg.angular_velocity.z

    # rospy.loginfo(f"runtime and torque: {imu_time: .1f}, {angular_velocity_z: .5f} ")
    if not stop_plotting_outer_tms_heading and outer_start_stop_initiator < start_stop_limit:
        time_list_first.append(imu_time)
        angular_velocity_z_list_first.append(angular_velocity_z)
    
    elif stop_plotting_outer_tms_heading:
        outer_start_stop_initiator = 8

    if start_plotting_inner_tms_heading and inner_start_stop_initiator < start_stop_limit:
        time_list_second.append(imu_time)
        angular_velocity_z_list_second.append(angular_velocity_z)

    elif stop_plotting_inner_tms_heading:
        inner_start_stop_initiator = 8

def plot_data():
    # rospy.loginfo("Plotting data")
    # plt.figure(figsize= (12,8))
    # plt.plot(time_list_first, angular_velocity_z_list_first, color = "black", marker = "." , label = "")
    # plt.title("TMS angular velocity in z axis")
    # plt.xlabel("simulation time (sec)")
    # plt.ylabel("angular velocity (rad/sec)")
    # plt.grid()
    # plt.legend(loc = "upper left")
    # plt.show()

    # fig, ax = plt.subplots(figsize= (8,6))

    fig, ax = plt.subplots(2,1, figsize= (14,10), sharey = False)
    fig.suptitle("Sharing both axes")
    ax[0].set_title("First itteration of TMS heading - angular velocity in z axis")
    ax[0].set_ylabel("angular velocity (rad/sec)")
    ax[0].set_xlabel("simulation time (sec)")
    ax[0].grid()
    ax[0].legend(loc = "upper left")
    ax[1].set_title("Second itteration of TMS heading - angular velocity in z axis")
    ax[1].set_ylabel("angular velocity (rad/sec)")
    ax[1].set_xlabel("simulation time (sec)")
    ax[1].grid()
    ax[1].legend(loc = "upper left")

    ax[0].plot(time_list_first, angular_velocity_z_list_first, color = "blue", marker = "." , label = "")
    ax[1].plot(time_list_second, angular_velocity_z_list_second, color = "red", marker = "." , label = "")
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

if __name__ == "__main__":
    rospy.init_node("time_and_angular_velocity_z")

    # rospy.Subscriber("/clock", Clock, time_callback)

    # rospy.Subscriber("/tms/imu", Imu, time_callback)
    # rospy.Subscriber("/tms/imu", Imu, angular_velocity_z_callback)
    rospy.Subscriber("/tms/imu", Imu, time_and_angular_velocity_callback)

    rospy.Subscriber("/tms/heading_done", Bool, stop_outer_heading_call)
    rospy.Subscriber("/rov/approach_done", Bool, start_inner_heading_call)
    rospy.Subscriber("/tms/inner_heading_done", Bool, stop_inner_heading_call)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()