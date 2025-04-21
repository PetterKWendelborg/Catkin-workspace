#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from matplotlib import pyplot as plt

time_list = []
linear_acceleration_x_list = []

def time_and_linear_acceleration_callback(msg):
    global time_list
    global linear_velocity_x_list


    imu_time = msg.header.stamp.to_sec()
    linear_acceleration_x = msg.linear_acceleration.x
    
    # rospy.loginfo(f"runtime and torque: {imu_time: .1f}, {angular_velocity_z: .5f} ")
    time_list.append(imu_time)
    linear_acceleration_x_list.append(linear_acceleration_x)

def plot_data():

    plt.figure(figsize= (12,8))
    plt.plot(time_list, linear_acceleration_x_list, color = "black", marker = "." , label = "")
    plt.title("ROV linear acceleration in z axis")
    plt.xlabel("simulation time (sec)")
    plt.ylabel("linear acceleration (m/s²)")
    plt.grid()
    plt.legend(loc = "upper left")
    plt.show()

if __name__ == "__main__":
    rospy.init_node("time_and_angular_velocity_z")

    # rospy.Subscriber("/clock", Clock, time_callback)

    # rospy.Subscriber("/tms/imu", Imu, time_callback)
    # rospy.Subscriber("/tms/imu", Imu, angular_velocity_z_callback)
    rospy.Subscriber("/tms/imu", Imu, time_and_linear_acceleration_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()