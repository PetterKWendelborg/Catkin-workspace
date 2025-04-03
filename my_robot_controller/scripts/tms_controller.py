#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Twist

def center_call(center_msg, pub):

    center_x = center_msg.x
    center_z = center_msg.z
    hyst_window = 5.0

    if center_z != 0:
        angle_to_center_gaz = math.asin(center_x/center_z)
    else:
        angle_to_center_gaz = 0

    angle_to_center_gaz_deg = math.degrees(angle_to_center_gaz)
    # rospy.loginfo(f"angle to center = {angle_to_center_gaz} rad")  
    # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center_gaz)} deg") 

    tms_twist_msg = Twist()

    if angle_to_center_gaz_deg >= -hyst_window and angle_to_center_gaz_deg <= hyst_window:
        tms_twist_msg.angular.z = 0

    elif angle_to_center_gaz_deg <= -hyst_window:
        tms_twist_msg.angular.z = 0.5

    elif angle_to_center_gaz_deg >= hyst_window:
        tms_twist_msg.angular.z = -0.5

    else:
        tms_twist_msg.angular.z = 0


    # tms_twist_msg.angular.z = angle_to_center_gaz

    pub.publish(tms_twist_msg)


if __name__ == "__main__":
    rospy.init_node("tms_heading")
    
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

    rospy.Subscriber("/rov_center", Point, center_call, cmd_vel_pub)

    rospy.spin()

