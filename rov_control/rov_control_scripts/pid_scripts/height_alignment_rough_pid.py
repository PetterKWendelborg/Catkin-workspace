#!/usr/bin/env python3 
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PointStamped

# This code subscribes to the /rov/tms_center topic which gets its information from pcdata_conv_rov.py
# and to the /tms/heading_done topic which gets its information from tms_heading_2d/3d(_new).py
# here the sonar data is finaly used to adjust the ROV's heading towards the TMS

height_alignment_ready = False
last_time_in_window = None

def center_call(center_msg, pub):
    center_y = center_msg.point.y

    upper_hyst_window = 0
    bottom_hyst_window = -0.3

    high_hyst_window = 0.2
    low_hyst_window = -0.5

    hysteresis_duration = 4

    now = rospy.get_time()

    inner_force = 0.5
    outer_force = 1.5

    global last_time_in_window
    global height_alignment_ready
    
    if height_alignment_ready:

        rov_twist_msg= Twist()
        boolean = Bool()

        if center_y <= upper_hyst_window and center_y >= bottom_hyst_window:
            rov_twist_msg.linear.z = 0
            last_time_in_window = now
            pub.publish(rov_twist_msg)
            while last_time_in_window != None:
                now = rospy.get_time()
                if now - last_time_in_window >= hysteresis_duration:
                    # Stayed in window for enough time
                    boolean.data = True
                    condition_pub.publish(boolean)
                    rospy.signal_shutdown("Condition met")

        # om høyden er høyere enn 0.5
        elif center_y > high_hyst_window:
            rov_twist_msg.linear.z = -outer_force
            last_time_in_window = None

        # om høyden er mellom 0 og 0.5
        elif center_y < high_hyst_window and center_y > upper_hyst_window :
            rov_twist_msg.linear.z = -inner_force
            last_time_in_window = None

        # om høyden er lavere enn -1
        elif center_y < low_hyst_window:
            rov_twist_msg.linear.z = outer_force
            last_time_in_window = None

        # om høyden er mellom -1 og -0.5
        elif center_y > low_hyst_window and center_y < bottom_hyst_window :
            rov_twist_msg.linear.z = inner_force
            last_time_in_window = None

        else:
            rospy.loginfo("else statement")
            rov_twist_msg.linear.y = 0

        rospy.loginfo(center_y)
        pub.publish(rov_twist_msg)

def condition_call(rov_heading_done):
    global height_alignment_ready
    if rov_heading_done.data:
        height_alignment_ready = rov_heading_done.data
        
if __name__ == "__main__":
    rospy.init_node("rov_heading")
    
    cmd_vel_pub = rospy.Publisher("/rov/cmd_vel", Twist, queue_size = 10)
    
    rospy.Subscriber("/rov/tms_center", PointStamped, center_call, cmd_vel_pub)
    rospy.Subscriber("/rov/heading_done", Bool, condition_call)

    condition_pub = rospy.Publisher("/rov/far_alignment_done", Bool, queue_size = 10)

    rospy.spin()