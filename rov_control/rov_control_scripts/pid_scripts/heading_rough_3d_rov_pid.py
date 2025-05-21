#!/usr/bin/env python3 
import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PointStamped

# This code subscribes to the /rov/tms_center topic which gets its information from pcdata_conv_rov.py
# and to the /tms/heading_done topic which gets its information from tms_heading_2d/3d(_new).py
# here the sonar data is finaly used to adjust the ROV's heading towards the TMS

tms_heading_done = False
last_time_in_window = None

def center_call(center_msg, pub):
    center_x = center_msg.point.x
    center_z = center_msg.point.z
    inner_hyst_window = 1
    outer_hyst_window = 10.0
    hysteresis_duration = 4
    now = rospy.get_time()

    inner_torque = 0.6
    outer_torque = 2.0

    global last_time_in_window
    global tms_heading_done
    if tms_heading_done:
        
        if center_z != 0:
            angle_to_center = math.asin(center_x/center_z)

        else:
            angle_to_center = 0

        angle_to_center_gaz_deg = math.degrees(angle_to_center)

        rov_twist_msg = Twist()
        boolean = Bool()

        if angle_to_center_gaz_deg >= -inner_hyst_window and angle_to_center_gaz_deg <= inner_hyst_window:
            rov_twist_msg.angular.z = 0
            last_time_in_window = now
            pub.publish(rov_twist_msg)
            while last_time_in_window != None:
                now = rospy.get_time()
                if now - last_time_in_window >= hysteresis_duration:
                    # Stayed in window for enough time
                    boolean.data = True
                    condition_pub.publish(boolean)
                    rospy.signal_shutdown("Condition met")

        # angle is < -15
        elif angle_to_center_gaz_deg <= -outer_hyst_window:
            rov_twist_msg.angular.z = outer_torque
            last_time_in_window = None

        # -15 < angle > -0.5
        elif angle_to_center_gaz_deg > -outer_hyst_window and angle_to_center_gaz_deg < -inner_hyst_window :
            rov_twist_msg.angular.z = inner_torque
            last_time_in_window = None

        # angle > 15
        elif angle_to_center_gaz_deg >= outer_hyst_window:
            rov_twist_msg.angular.z = -outer_torque
            last_time_in_window = None

        #  15 > angle > 0.3
        elif angle_to_center_gaz_deg < outer_hyst_window and angle_to_center_gaz_deg > inner_hyst_window :
            rov_twist_msg.angular.z = -inner_torque
            last_time_in_window = None

        else:
            rospy.loginfo("else statement")
            rov_twist_msg.angular.z = 0

        pub.publish(rov_twist_msg)

def condition_call(approach_ready):
    global tms_heading_done
    if approach_ready.data:
        tms_heading_done = approach_ready.data
        
if __name__ == "__main__":
    rospy.init_node("rov_heading_pid")
    
    cmd_vel_pub = rospy.Publisher("/rov/cmd_vel", Twist, queue_size = 10)

    rospy.Subscriber("/rov/tms_center", PointStamped, center_call, cmd_vel_pub)
    rospy.Subscriber("/tms/heading_done", Bool, condition_call)

    condition_pub = rospy.Publisher("/rov/heading_done", Bool, queue_size = 10)

    rospy.spin()