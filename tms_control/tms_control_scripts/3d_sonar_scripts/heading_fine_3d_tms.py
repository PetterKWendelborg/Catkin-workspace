#!/usr/bin/env python3 
import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Wrench, PointStamped

# This code subscribes to the /tms/rov_center topic which gets its information from ponitcloud_to_xyz_3d_tms.py
# here the sonar data is finaly used to adjust the TMS's heading towards the ROV

tms_inner_heading_ready = False
last_time_in_window = None

def center_call(center_msg, pub):
    center_x = center_msg.point.x
    center_z = center_msg.point.z
    inner_hyst_window = 0.5
    outer_hyst_window = 10.0
    hysteresis_duration = 5
    now = rospy.get_time()

    inner_torque = 0.1
    outer_torque = 1.0

    global last_time_in_window
    global tms_inner_heading_ready
    if tms_inner_heading_ready:
        
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

        # If the angle is less than -outer_hyst_window
        elif angle_to_center_gaz_deg <= -outer_hyst_window:
            tms_wrench_msg.torque.z = outer_torque
            last_time_in_window = None

        # If the angle is betweem -outer_hyst_window and -inner_hyst_window 
        elif angle_to_center_gaz_deg > -outer_hyst_window and angle_to_center_gaz_deg < -inner_hyst_window :
            tms_wrench_msg.torque.z = inner_torque
            last_time_in_window = None

        # If the angle is higher than outer_hyst_window
        elif angle_to_center_gaz_deg >= outer_hyst_window:
            tms_wrench_msg.torque.z = -outer_torque
            last_time_in_window = None

        # If the angle is betweem outer_hyst_window and inner_hyst_window 
        elif angle_to_center_gaz_deg < outer_hyst_window and angle_to_center_gaz_deg > inner_hyst_window :
            tms_wrench_msg.torque.z = -inner_torque
            last_time_in_window = None

        else:
            rospy.loginfo("else statement")
            tms_wrench_msg.torque.z = 0

        pub.publish(tms_wrench_msg)
    
def condition_call(approach_done):
    global tms_inner_heading_ready
    if approach_done.data:
        tms_inner_heading_ready = approach_done.data

if __name__ == "__main__":
    rospy.init_node("tms_inner_heading")
    
    cmd_vel_pub = rospy.Publisher("/tms/thruster_manager/input", Wrench, queue_size = 10)

    rospy.Subscriber("/tms/rov_center", PointStamped, center_call, cmd_vel_pub)
    rospy.Subscriber("/rov/approach_done", Bool, condition_call)

    condition_pub = rospy.Publisher("/tms/inner_heading_done", Bool, queue_size = 10)

    rospy.spin()