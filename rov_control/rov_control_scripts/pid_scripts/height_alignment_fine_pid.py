#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool

inner_rov_heading_done = False 
aruco_detected = False
height_alignment_ready = False
last_time_in_window = None

def start_docking(aruco_msg, pub):
    # global condition
    distance_y_rov_to_aruco_4x4 = aruco_msg.point.y
    hysteresis_duration = 6
    terminal_center_hysterisis = 0.002
    force_y = 0.1
    rov_twist_msg = Twist()
    boolean = Bool()
    now = rospy.get_time()
    global aruco_detected
    global last_time_in_window
    global inner_rov_heading_done
 
    if height_alignment_ready and aruco_detected:
        # aruco_msg uses NED(North East Down) frame, whilst rov_wrench_msg uses normal frame
        # If ths ROV is above the TMS opening
        if distance_y_rov_to_aruco_4x4 >= -terminal_center_hysterisis and distance_y_rov_to_aruco_4x4 < terminal_center_hysterisis:
            rospy.loginfo("inside_hyst_window")
            rov_twist_msg.linear.z = 0 
            last_time_in_window = now
            pub.publish(rov_twist_msg) 
            while last_time_in_window != None:
                now = rospy.get_time()
                if now - last_time_in_window >= hysteresis_duration:
                    # Stayed in window for enough time
                    boolean.data = True
                    docking_pub.publish(boolean)
                    rospy.signal_shutdown("Condition met")
                
        elif distance_y_rov_to_aruco_4x4 < -terminal_center_hysterisis:
            rov_twist_msg.linear.z = force_y     # Go downwards
            pub.publish(rov_twist_msg) 

        # If the ROV is below the TMS opening
        elif distance_y_rov_to_aruco_4x4 > terminal_center_hysterisis:
            rov_twist_msg.linear.z = -force_y      # Go upwards
            pub.publish(rov_twist_msg) 

        # If the ROV is in position to continue docking
        else:
            rov_twist_msg.linear.z = 0       # Stand still
            pub.publish(rov_twist_msg) 

    elif not height_alignment_ready and not aruco_detected:
        rospy.loginfo(f"1 - start_condition: {height_alignment_ready} aruco_detected_4x4: {aruco_detected}")

    elif height_alignment_ready and  not aruco_detected:
        rov_twist_msg.linear.z = 0  
        rospy.loginfo(f"2 - start_condition: {height_alignment_ready} aruco_detected_4x4: {aruco_detected}")
        pub.publish(rov_twist_msg) 

    else:
        rospy.loginfo(f"3 - start_condition: {height_alignment_ready} aruco_detected_4x4: {aruco_detected}")
    
def condition_call(inner_rov_heading_done):
    global height_alignment_ready
    if inner_rov_heading_done.data:
        height_alignment_ready = inner_rov_heading_done.data

def aruco_call(aruco_msg):
    global aruco_detected
    aruco_detected = aruco_msg.data

if __name__ == "__main__":
    rospy.init_node("height_alignment_pid")

    cmd_vel_pub = rospy.Publisher("/rov/cmd_vel", Twist, queue_size = 10)
    
    rospy.Subscriber("/rov/inner_heading_done", Bool, condition_call)
    rospy.Subscriber("/rov/aruco_detect_4x4", Bool, aruco_call)
    rospy.Subscriber("/rov/aruco_4x4", PointStamped, start_docking, cmd_vel_pub)
    
    docking_pub = rospy.Publisher("/rov/alignment_done", Bool, queue_size = 10)

    rospy.spin()