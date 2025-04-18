#!/usr/bin/env python3 
import rospy
import math
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Wrench

last_time_in_window = None

def center_call(center_msg, pub):
    global last_time_in_window
    center_x = center_msg.x
    center_z = center_msg.z
    hyst_window = 3
    hysteresis_duration = 4
    now = rospy.get_time()

    if center_z != 0:
        angle_to_center = math.asin(center_x/center_z)
    else:
        angle_to_center = 0

    angle_to_center_gaz_deg = math.degrees(angle_to_center)
    # rospy.loginfo(f"angle to center = {angle_to_center_gaz} rad")  
    # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center_gaz)} deg") 

    tms_wrench_msg = Wrench()
    boolean = Bool()

    if angle_to_center_gaz_deg >= -hyst_window and angle_to_center_gaz_deg <= hyst_window:
        tms_wrench_msg.torque.z = 0
        

        if last_time_in_window is None:
            last_time_in_window = now

        elif now - last_time_in_window >= hysteresis_duration:
            # Stayed in window for enough time
            rospy.loginfo("plz_head_rov")
            boolean.data = True
            condition_pub.publish(boolean)
            rospy.signal_shutdown("Condition met")

    elif angle_to_center_gaz_deg <= -hyst_window:
        tms_wrench_msg.torque.z = 4
        
        last_time_in_window = None

    elif angle_to_center_gaz_deg >= hyst_window:
        tms_wrench_msg.torque.z = -4
        
        last_time_in_window = None

    else:
        tms_wrench_msg.torque.z = 0
    
        last_time_in_window = None
    pub.publish(tms_wrench_msg)
if __name__ == "__main__":
    rospy.init_node("tms_heading")
    
    cmd_vel_pub = rospy.Publisher("/tms/thruster_manager/input", Wrench, queue_size = 10)

    rospy.Subscriber("/tms/rov_center", Point, center_call, cmd_vel_pub)
    condition_pub = rospy.Publisher("/tms/heading_done", Bool, queue_size = 10)

    rospy.spin()
