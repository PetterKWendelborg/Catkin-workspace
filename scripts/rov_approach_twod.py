#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench
import time
from std_msgs.msg import Bool
rov_heading_done = False
last_time_in_window = None

def center_call(center_msg, pub):
    # global condition
    center_x = center_msg.x
    center_y = center_msg.y
    # rospy.loginfo(f"{center_x}, {center_y}")

    buffer_distance = 0.5
    desired_distance = 1.5
    stopping_distance = desired_distance + buffer_distance
    hysteresis_duration = 4
    now = rospy.get_time()
    global last_time_in_window
    global rov_heading_done

    if rov_heading_done:

        rov_wrench_msg = Wrench()
        boolean = Bool()

        if center_x <= stopping_distance:
            rov_wrench_msg.force.x = 0

            if last_time_in_window is None:
                last_time_in_window = now

            elif now - last_time_in_window >= hysteresis_duration:
                # Stayed in window for enough time
                rospy.loginfo("plz_dock_rov")
                boolean.data = True
                approach_pub.publish(boolean)
                rospy.signal_shutdown("Condition met")

        elif center_x > stopping_distance:
            rov_wrench_msg.force.x = 2

        else:
            rov_wrench_msg.force.x = 0

        pub.publish(rov_wrench_msg)
            
    else:
        rospy.loginfo(f"condition is {rov_heading_done}")
    

def condition_call(approach_ready):
    global rov_heading_done
    if approach_ready.data:
        rov_heading_done = approach_ready.data

if __name__ == "__main__":
    rospy.init_node("rov_heading")
    
    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    rospy.Subscriber("/rov/heading_done", Bool, condition_call)

    rospy.Subscriber("/rov/tms_center", Point, center_call, cmd_vel_pub)

    approach_pub = rospy.Publisher("/rov/aproach_done", Bool, queue_size = 10)
    rospy.spin()