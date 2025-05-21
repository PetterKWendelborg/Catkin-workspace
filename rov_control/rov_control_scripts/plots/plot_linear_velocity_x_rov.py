#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from matplotlib import pyplot as plt

time_list = []
linear_velocity_x_list = []

# linear_acceleration_x_list = []
# current_velocity_x = [0.0]
linear_velocity_x = 0.0
last_time = None

def time_and_linear_acceleration_callback(msg):
    global time_list
    global linear_velocity_x_list
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
    rospy.loginfo(f"\ndelta_time: {delta_time} \ncurrent_time: {current_time}\nlast_time: {last_time} \nlinear_velocity_x:{linear_velocity_x} \nlinear_acceleration_x:{linear_acceleration_x}\n")

    time_list.append(current_time)
    linear_velocity_x_list.append(linear_velocity_x)
    # linear_velocity_x_list.append(linear_acceleration_x)

def plot_data():

    plt.figure(figsize= (12,8))
    plt.plot(time_list, linear_velocity_x_list, color = "black", marker = "." , label = "")
    plt.title("ROV linear velociity in x axis")
    plt.xlabel("simulation time (sec)")
    plt.ylabel("linear velocity (m/s)")
    plt.grid()
    plt.legend(loc = "upper left")
    plt.show()

if __name__ == "__main__":
    rospy.init_node("ROV_linear_velocity_x")

    rospy.Subscriber("/rov/imu", Imu, time_and_linear_acceleration_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()