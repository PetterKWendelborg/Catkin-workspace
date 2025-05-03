#!/usr/bin/env python3 
import rospy
import math
from matplotlib import pyplot as plt
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

time_list_first = []
heading_angle_list_list_first = []

time_list_second = []
heading_angle_list_list_second = []


#Det jeg trenger er å vite når den tms heading starter og når den slutter. Som er når den får en verdi til å starte, og sender en ny verdi
stop_plotting_outer_tms_heading = False
start_plotting_inner_tms_heading = False
stop_plotting_inner_tms_heading = False
display_plot = False

def heading_angle_callback(msg):
    global time_list_first
    global heading_angle_list_list_first
    global time_list_second
    global heading_angle_list_list_second
    
    heading_angle_time = msg.header.stamp.to_sec()
    heading_angle_value = msg.point.z
    
    # rospy.loginfo(f"haeding time and angle value: {heading_angle_time}, {heading_angle_value} ")
    if not stop_plotting_outer_tms_heading:
        # rospy.loginfo(heading_angle_value)
        time_list_first.append(heading_angle_time)
        heading_angle_list_list_first.append(heading_angle_value)

    if start_plotting_inner_tms_heading and not stop_plotting_inner_tms_heading:
        time_list_second.append(heading_angle_time)
        heading_angle_list_list_second.append(heading_angle_value)

def plot_data():

    fig, ax = plt.subplots(2,1, figsize= (12,8), sharey = False)
    fig.canvas.set_window_title("plot_heading_yaw_angle_tms.py")
    fig.suptitle("")
    ax[0].set_title("First itteration of TMS heading - TMS heading angle")
    ax[0].set_ylabel("angle (deg)")
    ax[0].set_xlabel("simulation time (sec)")
    ax[0].grid()
    ax[0].legend(loc = "upper left")
    ax[1].set_title("Second itteration of TMS heading - TMS heading angle")
    ax[1].set_ylabel("angle (deg)")
    ax[1].set_xlabel("simulation time (sec)")
    ax[1].grid()
    ax[1].legend(loc = "upper left")

    ax[0].plot(time_list_first, heading_angle_list_list_first, color = "blue", marker = "." , label = "")
    ax[1].plot(time_list_second, heading_angle_list_list_second, color = "red", marker = "." , label = "")
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
    rospy.init_node("tms_heading_angle")

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