#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench
from std_msgs.msg import Bool
import time

inner_rov_heading_done = False 
aruco_detected = False
height_alignment_ready = False
last_time_in_window = None

def start_docking(aruco_msg, pub):
    # global condition
    distance_y_rov_to_aruco_4x4 = aruco_msg.y
    hysteresis_duration = 4
    terminal_center_hysterisis = 0.04
    force_y = 1
    rov_wrench_msg = Wrench()
    boolean = Bool()
    now = rospy.get_time()
    global aruco_detected
    global last_time_in_window #denne brukes ikke enda, men legger den vel inn en timer slik at den står stille før den starter docking
    global inner_rov_heading_done

    if height_alignment_ready and aruco_detected:
        # rospy.loginfo("aruco detected")

        # aruco_msg uses NED(North East Down) frame, whilst rov_wrench_msg uses normal frame
        # If ths ROV is above the TMS opening
        if distance_y_rov_to_aruco_4x4 >= -terminal_center_hysterisis and distance_y_rov_to_aruco_4x4 < terminal_center_hysterisis:
            rov_wrench_msg.force.z = 0 

            if last_time_in_window is None:
                last_time_in_window = now

            elif now - last_time_in_window >= hysteresis_duration:
                # Stayed in window for enough time
                rospy.loginfo("plz_dock_rov")
                boolean.data = True
                docking_pub.publish(boolean)
                rospy.signal_shutdown("Condition met")
                
        elif distance_y_rov_to_aruco_4x4 < -terminal_center_hysterisis:
            rov_wrench_msg.force.z = force_y     # Go downwards

        # If the ROV is below the TMS opening
        elif distance_y_rov_to_aruco_4x4 > terminal_center_hysterisis:
            rov_wrench_msg.force.z = -force_y      # Go upwards

        # If the ROV is in position to continue docking
        else:
            rov_wrench_msg.force.z = 0       # Stand still

    elif not height_alignment_ready and not aruco_detected:
        rov_wrench_msg.force.z = 0  
        rospy.loginfo(f"1 - start_condition: {height_alignment_ready} aruco_detected_4x4: {aruco_detected}")
    elif height_alignment_ready and  not aruco_detected:
        rov_wrench_msg.force.z = 0  
        rospy.loginfo(f"2 - start_condition: {height_alignment_ready} aruco_detected_4x4: {aruco_detected}")

    else:
        rospy.loginfo(f"3 - start_condition: {height_alignment_ready} aruco_detected_4x4: {aruco_detected}")

    pub.publish(rov_wrench_msg) 


def condition_call(inner_rov_heading_done):
    global height_alignment_ready
    if inner_rov_heading_done.data:
        height_alignment_ready = inner_rov_heading_done.data

def aruco_call(aruco_msg):
    global aruco_detected
    aruco_detected = aruco_msg.data

if __name__ == "__main__":
    rospy.init_node("height_alignment")

    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    rospy.Subscriber("/rov/inner_heading_done", Bool, condition_call)
    rospy.Subscriber("/rov/aruco_detect_4x4", Bool, aruco_call)
    rospy.Subscriber("/rov/aruco_4x4", Point, start_docking, cmd_vel_pub)
    
    docking_pub = rospy.Publisher("/rov/alignment_done", Bool, queue_size = 10)

    rospy.spin()