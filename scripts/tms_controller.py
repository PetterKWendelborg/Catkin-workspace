#!/usr/bin/env python3 
import rospy
import math
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Wrench

def center_call(center_msg, pub):

    center_x = center_msg.x
    center_y = center_msg.y
    hyst_window = 3.0

    if center_y != 0:
        angle_to_center = math.asin(center_y/center_x)
    else:
        angle_to_center = 0

    angle_to_center_gaz_deg = math.degrees(angle_to_center)
    # rospy.loginfo(f"angle to center = {angle_to_center_gaz} rad")  
    # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center_gaz)} deg") 

    tms_wrench_msg = Wrench()
    boolean = Bool()

    if angle_to_center_gaz_deg >= -hyst_window and angle_to_center_gaz_deg <= hyst_window:
        tms_wrench_msg.torque.z = 0
        time.sleep(5)
        if angle_to_center_gaz_deg >= -hyst_window and angle_to_center_gaz_deg <= hyst_window:
            rospy.loginfo("pls_head_rov")
            boolean.data = True
            pub.publish(boolean)

    elif angle_to_center_gaz_deg <= -hyst_window:
        tms_wrench_msg.torque.z = -4

    elif angle_to_center_gaz_deg >= hyst_window:
        tms_wrench_msg.torque.z = 4

    else:
        tms_wrench_msg.torque.z = 0


    # tms_twist_msg.angular.z = angle_to_center_gaz

    pub.publish(tms_wrench_msg)


if __name__ == "__main__":
    rospy.init_node("tms_heading")
    
    cmd_vel_pub = rospy.Publisher("/tms/thruster_manager/input", Wrench, queue_size = 10)

    heading_done_pub = rospy.Publisher("/tms/heading_done", Bool, queue_size = 10)

    rospy.Subscriber("/tms/rov_center", Point, center_call, cmd_vel_pub)

    rospy.spin()

