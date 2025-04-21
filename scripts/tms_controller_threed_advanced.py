#!/usr/bin/env python3 
import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Wrench

last_time_in_window = None

def center_call(center_msg, pub):
    global last_time_in_window
    center_x = center_msg.x
    center_z = center_msg.z
    inner_hyst_window = 1.5
    outer_hyst_window = 6.0
    hysteresis_duration = 4

    now = rospy.get_time()

    inner_torque = 1
    outer_torque = 4

    if center_z != 0:
        angle_to_center = math.asin(center_x/center_z)
    else:
        angle_to_center = 0

    angle_to_center_gaz_deg = math.degrees(angle_to_center)
    # rospy.loginfo(f"angle to center = {angle_to_center_gaz} rad")  
    # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center_gaz)} deg") 

    tms_wrench_msg = Wrench()
    boolean = Bool()

    if angle_to_center_gaz_deg >= -inner_hyst_window and angle_to_center_gaz_deg <= inner_hyst_window:
        tms_wrench_msg.torque.z = 0

        if last_time_in_window is None:
            last_time_in_window = now

        elif now - last_time_in_window >= hysteresis_duration:
            # Stayed in window for enough time
            rospy.loginfo("plz_head_rov")
            boolean.data = True
            condition_pub.publish(boolean)
            rospy.signal_shutdown("Condition met")

    # om vinkel er mindre enn -10
    elif angle_to_center_gaz_deg <= -outer_hyst_window:
        tms_wrench_msg.torque.z = outer_torque
        last_time_in_window = None

    # om vinkel er mer enn -10 og mindre enn -3
    elif angle_to_center_gaz_deg > -outer_hyst_window and angle_to_center_gaz_deg < -inner_hyst_window :
        tms_wrench_msg.torque.z = inner_torque
        last_time_in_window = None

    # om vinkel er mer enn 10
    elif angle_to_center_gaz_deg >= outer_hyst_window:
        tms_wrench_msg.torque.z = -outer_torque
        last_time_in_window = None

    # om vinkel er mindre enn 10 og mer enn 3
    elif angle_to_center_gaz_deg < outer_hyst_window and angle_to_center_gaz_deg > inner_hyst_window :
        tms_wrench_msg.torque.z = -inner_torque
        last_time_in_window = None

    else:
        tms_wrench_msg.torque.z = 0


    pub.publish(tms_wrench_msg)


if __name__ == "__main__":
    rospy.init_node("tms_heading")
    
    cmd_vel_pub = rospy.Publisher("/tms/thruster_manager/input", Wrench, queue_size = 10)

    rospy.Subscriber("/tms/rov_center", Point, center_call, cmd_vel_pub)

    condition_pub = rospy.Publisher("/tms/heading_done", Bool, queue_size = 10)

    rospy.spin()