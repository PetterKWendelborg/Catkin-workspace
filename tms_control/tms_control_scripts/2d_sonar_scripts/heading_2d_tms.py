#!/usr/bin/env python3 
import rospy
import math
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Wrench

last_time_in_window = None
def center_call(center_msg, pub):
    center_x = center_msg.point.x
    center_y = center_msg.point.y
    hyst_window = 3.0
    outer_torque = 4
    hysteresis_duration = 5
    now = rospy.get_time()
    global last_time_in_window

    if center_y != 0:
        angle_to_center = math.asin(center_y/center_x)
    else:
        angle_to_center = 0

    angle_to_center_gaz_deg = math.degrees(angle_to_center)

    tms_wrench_msg = Wrench()
    boolean = Bool()

    if angle_to_center_gaz_deg >= -hyst_window and angle_to_center_gaz_deg <= hyst_window:
        tms_wrench_msg.torque.z = 0
        
        if last_time_in_window is None:
                last_time_in_window = now

        elif now - last_time_in_window >= hysteresis_duration:
            rospy.loginfo("pls_head_rov")
            boolean.data = True
            heading_done_pub.publish(boolean)
            rospy.signal_shutdown("Condition met")

    elif angle_to_center_gaz_deg <= -hyst_window:
        tms_wrench_msg.torque.z = -outer_torque
        last_time_in_window = None
    elif angle_to_center_gaz_deg >= hyst_window:
        tms_wrench_msg.torque.z = outer_torque
        last_time_in_window = None

    else:
        tms_wrench_msg.torque.z = 0
        last_time_in_window = None

    pub.publish(tms_wrench_msg)


if __name__ == "__main__":
    rospy.init_node("tms_heading")
    
    cmd_vel_pub = rospy.Publisher("/tms/thruster_manager/input", Wrench, queue_size = 10)
    heading_done_pub = rospy.Publisher("/tms/heading_done", Bool, queue_size = 10)
    rospy.Subscriber("/tms/rov_center", PointStamped, center_call, cmd_vel_pub)
    
    rospy.spin()

