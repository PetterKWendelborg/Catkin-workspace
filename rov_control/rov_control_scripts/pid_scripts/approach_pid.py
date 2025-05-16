#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool

# This code subscribes to the /rov/tms_center topic which gets its information from pcdata_conv_rov.py
# and to the /rov/heading_done topic which gets its information from rov_heading.py
# here the sonar data is used to make the ROV approach the TMS

approach_ready = False
last_time_in_window = None

def center_call(center_msg, pub):
    # global condition
    center_z = center_msg.point.z
    
    # rospy.loginfo(f"{center_z}")

    # buffer_distance = 0.28
    # desired_distance = 2.4
    # stopping_distance_outer = desired_distance
    # stopping_distance_inner = desired_distance + buffer_distance

    outer_hyst_window_range = 3

    middle_hyst_window_range = 2.4

    inner_hyst_window_range =  2.2

    outer_force = 5
    outer_middle_force = 3
    middle_inner_force = 1
    hysteresis_duration = 4
    now = rospy.get_time()
    global last_time_in_window
    global approach_ready

    if approach_ready:

        rov_twist_msg = Twist()
        boolean = Bool()

        #  If the distance is less than inner_hyst_window_range
        if center_z <= inner_hyst_window_range:
            rov_twist_msg.linear.x = 0
            last_time_in_window = now
            pub.publish(rov_twist_msg)
            while last_time_in_window != None:
                now = rospy.get_time()
                if now - last_time_in_window >= hysteresis_duration:
                    # Stayed in window for enough time
                    boolean.data = True
                    approach_pub.publish(boolean)
                    rospy.signal_shutdown("Condition met")

        # If the distance is larger/outside  oter_hyst_window_range
        elif center_z >= outer_hyst_window_range:
            rov_twist_msg.linear.x = outer_force

        # If the distance is between the middle_hyst_window_range and outer_hyst_window_range 
        elif center_z > middle_hyst_window_range and center_z < outer_hyst_window_range:
            rov_twist_msg.linear.x = outer_middle_force

        # If the distance is between the inner_hyst_window_range and middle_hyst_window_range  
        elif center_z > inner_hyst_window_range and center_z < middle_hyst_window_range:
            rov_twist_msg.linear.x = middle_inner_force

        else:
            rov_twist_msg.linear.x = 0

        pub.publish(rov_twist_msg)
            
    else:
        rospy.loginfo(f"condition is {approach_ready}")
    

def condition_call(far_alignment_done):
    global approach_ready
    if far_alignment_done.data:
        approach_ready = far_alignment_done.data

if __name__ == "__main__":
    rospy.init_node("rov_approach")
    
    cmd_vel_pub = rospy.Publisher("/rov/cmd_vel", Twist, queue_size = 10)
    
    rospy.Subscriber("/rov/far_alignment_done", Bool, condition_call)
    rospy.Subscriber("/rov/tms_center", PointStamped, center_call, cmd_vel_pub)

    approach_pub = rospy.Publisher("/rov/approach_done", Bool, queue_size = 10)
    rospy.spin()
