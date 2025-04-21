#!/usr/bin/env python3 
import rospy
import math
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from matplotlib import pyplot as plt

# This code makes a plot when the simulation is done from the tms yaw data from the IMU

time_list = []
angular_velocity_z_list = []

# def angular_velocity_z_callback(msg):
#     global torque_z_list
#     angular_velocity = msg.angular_velocity.z
#     angular_velocity_z_list.append(angular_velocity)
#     rospy.loginfo(f"angualar veloxity in z: {torque_z}")

#     rospy.loginfo(f"-----------------") 


# def time_callback(msg):
#     global time_list
#     # time = msg.clock.to_sec()
#     # time_list.append(time)
#     # rospy.loginfo(f"runtime: {time: .1f}")

#     imu_time = msg.header.stamp.to_sec()
#     # rospy.loginfo(f"runtime: {imu_time: .1f}")
#     time_list.append(imu_time)
#     rospy.loginfo(f"-----------------") 

#     rospy.loginfo(f"runtime: {time_list}")

def time_and_angular_velocity_callback(msg):
    global time_list
    global torque_z_list


    imu_time = msg.header.stamp.to_sec()
    angular_velocity_z = msg.angular_velocity.z
    
    # rospy.loginfo(f"runtime and torque: {imu_time: .1f}, {angular_velocity_z: .5f} ")
    time_list.append(imu_time)
    angular_velocity_z_list.append(angular_velocity_z)

def plot_data():

    plt.figure(figsize= (12,8))
    plt.plot(time_list, angular_velocity_z_list, color = "black", marker = "." , label = "")
    plt.title("")
    plt.xlabel("simulation time (sec)")
    plt.ylabel("angular velocity in z axis (rad/sec)")
    plt.grid()
    plt.legend(loc = "upper left")
    plt.show()

if __name__ == "__main__":
    rospy.init_node("time_and_angular_velocity_z")

    # rospy.Subscriber("/clock", Clock, time_callback)

    # rospy.Subscriber("/tms/imu", Imu, time_callback)
    # rospy.Subscriber("/tms/imu", Imu, angular_velocity_z_callback)
    rospy.Subscriber("/tms/imu", Imu, time_and_angular_velocity_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plot_data()