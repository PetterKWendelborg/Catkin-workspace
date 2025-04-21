#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu

angle_to_center_deg = 0.0
angular_velocity_z = 0.0

def center_call(center_msg):
    center_x = center_msg.x
    center_z = center_msg.z
    global angle_to_center_deg 

    if center_z != 0:
        angle_to_center = math.asin(center_x/center_z)
    else:
        angle_to_center = 0

    angle_to_center_deg = math.degrees(angle_to_center)
    log()
    
def angular_velocity_callback(msg):
    global angular_velocity_z 
    angular_velocity_z = msg.angular_velocity.z
    log()
    # rospy.loginfo(f"runtime and angular_velocity: {imu_time: .1f}, {angular_velocity_z: .5f} ")

def log():

    rospy.loginfo(f"angle and angular velocity: \n{angle_to_center_deg} deg \n{angular_velocity_z} rad/sec")

if __name__ == "__main__":
    rospy.init_node("angle_to_tms")
    
    rospy.Subscriber("/rov/tms_center", Point, center_call)
    rospy.Subscriber("/rov/imu", Imu, angular_velocity_callback)

    rospy.spin()