#!/usr/bin/env python3 
import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Wrench

# This code subscribes to the /rov/tms_center topic which gets its information from pcdata_conv_rov.py
# and to the /tms/heading_done topic which gets its information from tms_heading_2d/3d(_new).py
# here the sonar data is finaly used to adjust the ROV's heading towards the TMS

tms_heading_done = False
last_time_in_window = None

def center_call(center_msg, pub):
    center_x = center_msg.x
    center_z = center_msg.z
    inner_hyst_window = 0.5
    outer_hyst_window = 15.0
    hysteresis_duration = 4
    now = rospy.get_time()

    inner_torque = 0.1
    outer_torque = 1.5

    global last_time_in_window
    global tms_heading_done
    
    if tms_heading_done:
        
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
                rospy.loginfo("plz_move_forward_rov")
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
            rospy.loginfo("else statement")
            tms_wrench_msg.torque.z = 0

        pub.publish(tms_wrench_msg)

def condition_call(approach_ready):
    global tms_heading_done
    if approach_ready.data:
        tms_heading_done = approach_ready.data
        
if __name__ == "__main__":
    rospy.init_node("rov_heading")
    
    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    rospy.Subscriber("/rov/tms_center", Point, center_call, cmd_vel_pub)

    rospy.Subscriber("/tms/heading_done", Bool, condition_call)

    condition_pub = rospy.Publisher("/rov/heading_done", Bool, queue_size = 10)

    rospy.spin()