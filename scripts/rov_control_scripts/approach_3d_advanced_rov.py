#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench, PointStamped
import time
from std_msgs.msg import Bool

# This code subscribes to the /rov/tms_center topic which gets its information from pcdata_conv_rov.py
# and to the /rov/heading_done topic which gets its information from rov_heading.py
# here the sonar data is used to make the ROV approach the TMS

approach_ready = False
last_time_in_window = None

def center_call(center_msg, pub):
    # global condition
    center_z = center_msg.z
    
    # rospy.loginfo(f"{center_z}")

    buffer_distance = 0.28
    desired_distance = 2.8
    stopping_distance_outer = desired_distance
    stopping_distance_inner = desired_distance + buffer_distance
    outer_force = 5
    inner_force = 1.5
    hysteresis_duration = 4
    now = rospy.get_time()
    global last_time_in_window
    global approach_ready

    if approach_ready:

        rov_wrench_msg = Wrench()
        boolean = Bool()

        if center_z <= stopping_distance_inner:
            rov_wrench_msg.force.x = 0

            if last_time_in_window is None:
                last_time_in_window = now

            elif now - last_time_in_window >= hysteresis_duration:
                # Stayed in window for enough time
                rospy.loginfo("plz_dock_rov")
                boolean.data = True
                approach_pub.publish(boolean)
                rospy.signal_shutdown("Condition met")

        # If the rov are outside the outer distance window
        elif center_z >= stopping_distance_outer:
            rov_wrench_msg.force.x = outer_force

        # If the rov are between the outer and inner distance window
        elif center_z > stopping_distance_inner and center_z < stopping_distance_outer:
            rov_wrench_msg.force.x = inner_force
        else:
            rov_wrench_msg.force.x = 0

        pub.publish(rov_wrench_msg)
            
    else:
        rospy.loginfo(f"condition is {approach_ready}")
    

def condition_call(far_alignment_done):
    global approach_ready
    if far_alignment_done.data:
        approach_ready = far_alignment_done.data

if __name__ == "__main__":
    rospy.init_node("rov_approach")
    
    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    rospy.Subscriber("/rov/far_alignment_done", Bool, condition_call)

    rospy.Subscriber("/rov/tms_center", Point, center_call, cmd_vel_pub)

    approach_pub = rospy.Publisher("/rov/approach_done", Bool, queue_size = 10)
    rospy.spin()
