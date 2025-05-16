#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool

docking_ready = False
last_time_in_window = None

aruco_detected = False

def start_docking(aruco_msg, pub):
    # global condition
    # distance_rov_to_aruco_5x5 = aruco_msg.z

    distance_rov_to_aruco_5x5 = aruco_msg.point.z


    stopping_distance = 0.54
    terminal_opening_distance = 1.5

    outside_terminal_opening_hyst_range = terminal_opening_distance
    inside_terminal_opening_hyst_range = stopping_distance
    

    outer_force = 4
    inner_force = 2

    rov_twist_msg = Twist()
    boolean = Bool()
    global aruco_detected
    global last_time_in_window
    global docking_ready

    if  docking_ready and aruco_detected:

        #If ROV is past the inside_terminal_opening range
        if distance_rov_to_aruco_5x5 <= inside_terminal_opening_hyst_range:
            rospy.loginfo(f"inside stopping distance")
            # rospy.loginfo(f"aruco_detect: {aruco_detected}")
            # rospy.loginfo(f"boolean value:{boolean}")
            rov_twist_msg.linear.x = 0
            pub.publish(rov_twist_msg)
            boolean.data = True
            docking_pub.publish(boolean)
            rospy.loginfo(f"boolean value:{boolean}")


        #If ROV is outside the terminal opening range
        elif distance_rov_to_aruco_5x5 > outside_terminal_opening_hyst_range:
            rospy.loginfo(f"start thrusting")
            rov_twist_msg.linear.x = outer_force
            pub.publish(rov_twist_msg)

        #If ROV is between inside- and outsude terminal opening range
        elif distance_rov_to_aruco_5x5 > inside_terminal_opening_hyst_range and distance_rov_to_aruco_5x5 < outside_terminal_opening_hyst_range:
            rospy.loginfo(f"start thrusting")
            rov_twist_msg.linear.x = inner_force
            pub.publish(rov_twist_msg)


        else:
            rov_twist_msg.linear.x = 0
            rospy.loginfo(f"stop thrusting")
            pub.publish(rov_twist_msg)

    elif not docking_ready and not aruco_detected:
        rospy.loginfo(f"1 - start_condition: {docking_ready} aruco_detected_5x5: {aruco_detected}")

    elif docking_ready and not aruco_detected:
        rospy.loginfo(f"2 - start_condition: {docking_ready} aruco_detected_5x5: {aruco_detected}")
        rov_twist_msg.linear.x = 0
        pub.publish(rov_twist_msg)      
            
    else:
        rospy.loginfo(f"3 - start_condition: {docking_ready} aruco_detected_5x5: {aruco_detected}")

def fancy_docking_call(position_alignment_done):
    global docking_ready
    if position_alignment_done.data:
        docking_ready = position_alignment_done.data

def aruco_call(aruco_msg):
    global aruco_detected
    aruco_detected = aruco_msg.data

        

if __name__ == "__main__":
    rospy.init_node("rov_docking")

    cmd_vel_pub = rospy.Publisher("/rov/cmd_vel", Twist, queue_size = 10)
    
    rospy.Subscriber("/rov/alignment_done", Bool, fancy_docking_call)
    rospy.Subscriber("/rov/aruco_detect_5x5", Bool, aruco_call)
    # rospy.Subscriber("/rov/aruco_5x5", Point, start_docking, cmd_vel_pub)
    rospy.Subscriber("/rov/aruco_5x5", PointStamped, start_docking, cmd_vel_pub)

    docking_pub = rospy.Publisher("/rov/docking_done", Bool, queue_size = 10)

    rospy.spin()