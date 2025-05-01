#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench
from std_msgs.msg import Bool

docking_ready = False
last_time_in_window = None

aruco_detected = False

def start_docking(aruco_msg, pub):
    # global condition
    distance_rov_to_aruco_5x5 = aruco_msg.z

    #stopping distance må nok byttes ut senere bare setter den høy for testing
    stopping_distance = 0.55
    rov_wrench_msg = Wrench()
    boolean = Bool()
    global aruco_detected
    global last_time_in_window
    global docking_ready

    if  docking_ready and aruco_detected:

        if distance_rov_to_aruco_5x5 <= stopping_distance:
            rospy.loginfo(f"inside stopping distance")
            # rospy.loginfo(f"aruco_detect: {aruco_detected}")
            # rospy.loginfo(f"boolean value:{boolean}")
            rov_wrench_msg.force.x = 0
            pub.publish(rov_wrench_msg)
            boolean.data = True
            docking_pub.publish(boolean)
            rospy.loginfo(f"boolean value:{boolean}")

        elif distance_rov_to_aruco_5x5 > stopping_distance:
            rospy.loginfo(f"start thrusting")
            rov_wrench_msg.force.x = 2
            pub.publish(rov_wrench_msg)

        else:
            rov_wrench_msg.force.x = 0
            rospy.loginfo(f"stop thrusting")
            pub.publish(rov_wrench_msg)

    elif not docking_ready and not aruco_detected:
        rospy.loginfo(f"1 - start_condition: {docking_ready} aruco_detected_5x5: {aruco_detected}")
        rov_wrench_msg.force.x = 0
        pub.publish(rov_wrench_msg)

    elif docking_ready and not aruco_detected:
        rospy.loginfo(f"2 - start_condition: {docking_ready} aruco_detected_5x5: {aruco_detected}")
        rov_wrench_msg.force.x = 0
        pub.publish(rov_wrench_msg)      
            
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

    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    rospy.Subscriber("/rov/alignment_done", Bool, fancy_docking_call)
    rospy.Subscriber("/rov/aruco_detect_5x5", Bool, aruco_call)
    rospy.Subscriber("/rov/aruco_5x5", Point, start_docking, cmd_vel_pub)

    docking_pub = rospy.Publisher("/rov/docking_done", Bool, queue_size = 10)

    rospy.spin()