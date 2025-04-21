#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench
import time
from std_msgs.msg import Bool
tms_heading_done = False
last_time_in_window = None

def center_call(center_msg, pub):
    # global condition
    center_x = center_msg.x
    center_y = center_msg.y
    # rospy.loginfo(f"{center_x}, {center_y}")
    hyst_window = 3.0
    hysteresis_duration = 4
    now = rospy.get_time()
    global last_time_in_window
    global tms_heading_done

    if tms_heading_done:
        # rospy.loginfo("ready to turn")
        if center_y != 0:
            angle_to_center = math.asin(center_y/center_x)
        else:
            angle_to_center = 0

        angle_to_center_gaz_deg = math.degrees(angle_to_center)
        # rospy.loginfo(f"ready to turn{math.degrees(angle_to_center)}")

        rov_wrench_msg = Wrench()
        boolean = Bool()

        if angle_to_center_gaz_deg >= -hyst_window and angle_to_center_gaz_deg <= hyst_window:
            rov_wrench_msg.torque.z = 0

            if last_time_in_window is None:
                last_time_in_window = now

            elif now - last_time_in_window >= hysteresis_duration:
                # Stayed in window for enough time
                rospy.loginfo("plz_move_forward_rov")
                boolean.data = True
                tms_heading_done_pub.publish(boolean)
                rospy.signal_shutdown("Condition met")

        elif angle_to_center_gaz_deg <= -hyst_window:
            rov_wrench_msg.torque.z = -2
            last_time_in_window = None

        elif angle_to_center_gaz_deg >= hyst_window:
            rov_wrench_msg.torque.z = 2
            last_time_in_window = None
        else:
            rov_wrench_msg.torque.z = 0
            last_time_in_window = None
            
        pub.publish(rov_wrench_msg)
    
    else:
        rospy.loginfo(f"condition is {tms_heading_done}")

def condition_call(msg):
    global tms_heading_done
    if msg.data:
        tms_heading_done = msg.data


if __name__ == "__main__":
    rospy.init_node("rov_heading")
    
    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    rospy.Subscriber("/rov/tms_center", Point, center_call, cmd_vel_pub)
    
    rospy.Subscriber("/tms/heading_done", Bool, condition_call)
    tms_heading_done_pub = rospy.Publisher("/rov/heading_done", Bool, queue_size = 10)
    rospy.spin()

