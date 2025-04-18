#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench

def center_call(center_msg, pub):

    center_x = center_msg.x
    center_z = center_msg.z
    inner_hyst_window = 3.0
    outer_hyst_window = 10.0

    inner_torque = 2
    outer_torque = 4

    if center_z != 0:
        angle_to_center = math.asin(center_x/center_z)
    else:
        angle_to_center = 0

    angle_to_center_gaz_deg = math.degrees(angle_to_center)
    # rospy.loginfo(f"angle to center = {angle_to_center_gaz} rad")  
    # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center_gaz)} deg") 

    tms_wrench_msg = Wrench()

    if angle_to_center_gaz_deg >= -inner_hyst_window and angle_to_center_gaz_deg <= inner_hyst_window:
        tms_wrench_msg.torque.z = 0

    # om vinkel er mindre enn -10
    elif angle_to_center_gaz_deg <= -outer_hyst_window:
        tms_wrench_msg.torque.z = outer_torque

    # om vinkel er mer enn -10 og mindre enn -3
    elif angle_to_center_gaz_deg > -outer_hyst_window and angle_to_center_gaz_deg < -inner_hyst_window :
        tms_wrench_msg.torque.z = inner_torque

    # om vinkel er mer enn 10
    elif angle_to_center_gaz_deg >= outer_hyst_window:
        tms_wrench_msg.torque.z = -outer_torque

    # om vinkel er mindre enn 10 og mer enn 3
    elif angle_to_center_gaz_deg < outer_hyst_window and angle_to_center_gaz_deg > inner_hyst_window :
        tms_wrench_msg.torque.z = -inner_torque

    else:
        tms_wrench_msg.torque.z = 0


    pub.publish(tms_wrench_msg)


if __name__ == "__main__":
    rospy.init_node("tms_heading")
    
    cmd_vel_pub = rospy.Publisher("/tms/thruster_manager/input", Wrench, queue_size = 10)

    rospy.Subscriber("/tms/rov_center", Point, center_call, cmd_vel_pub)

    rospy.spin()