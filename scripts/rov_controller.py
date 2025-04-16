#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench
import time
from std_msgs.msg import Bool
condition = False
def center_call(center_msg, pub):
    global condition
    center_x = center_msg.x
    center_y = center_msg.y
    rospy.loginfo(f"{center_x}, {center_y}")
    hyst_window = 3.0
    if condition == True:
        

        if center_y != 0:
            angle_to_center = math.asin(center_y/center_x)
        else:
            angle_to_center = 0

        angle_to_center_gaz_deg = math.degrees(angle_to_center)
        # rospy.loginfo(f"angle to center = {angle_to_center_gaz} rad")  
        # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center_gaz)} deg") 

        rov_wrench_msg = Wrench()

        if angle_to_center_gaz_deg >= -hyst_window and angle_to_center_gaz_deg <= hyst_window:
            rov_wrench_msg.torque.z = 0

        elif angle_to_center_gaz_deg <= -hyst_window:
            rov_wrench_msg.torque.z = -4

        elif angle_to_center_gaz_deg >= hyst_window:
            rov_wrench_msg.torque.z = 4

        else:
            rov_wrench_msg.torque.z = 0

        pub.publish(rov_wrench_msg)
    
    else:
        rospy.loginfo(f"condition is {condition}")


    # rov_twist_msg.angular.z = angle_to_center_gaz


def condition_call(msg):
    global condition
    if msg.data:
        condition = msg.data


if __name__ == "__main__":
    rospy.init_node("rov_heading")
    
    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    rospy.Subscriber("/tms_center", Point, center_call, cmd_vel_pub)
    
    rospy.Subscriber("/tms/heading_done", Bool, condition_call)
    rospy.spin()

