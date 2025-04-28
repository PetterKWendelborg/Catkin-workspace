#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench
from std_msgs.msg import Bool

rov_approach_done = False
last_time_in_window = None

aruco_detected = False

def start_docking(aruco_msg, pub):
    # global condition
    distance_rov_to_aruco_4x4 = aruco_msg.y

    stopping_distance = 0.2
    rov_wrench_msg = Wrench()
    boolean = Bool()
    global aruco_detected
    global last_time_in_window
    global rov_approach_done

    if aruco_detected:
        rospy.loginfo(distance_rov_to_aruco_4x4)
        # aruco_msg uses NED(North East Down) frame, whilst rov_wrench_msg uses normal frame
        # If ths ROV is above the TMS opening
        if aruco_msg.y < -0.02:
            rov_wrench_msg.force.z = 1     # Go downwards

        # If the ROV is below the TMS opening
        elif aruco_msg.y > 0.02:
            rov_wrench_msg.force.z = -1      # Go upwards

        # If the ROV is in position to continue docking
        else:
            rov_wrench_msg.force.z = 0       # Stand still
     
    else:
        rov_wrench_msg.force.z = 0       # Stand still

    pub.publish(rov_wrench_msg)
    rospy.loginfo(type(distance_rov_to_aruco_4x4))

def aruco_call(aruco_msg):
    global aruco_detected
    aruco_detected = aruco_msg.data
        

if __name__ == "__main__":
    rospy.init_node("rov_docking")

    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    # rospy.Subscriber("/rov/approach_done", Bool, fancy_docking_call)
    rospy.Subscriber("/rov/aruco_detect_4x4", Bool, aruco_call)
    rospy.Subscriber("/rov/aruco_4x4", Point, start_docking, cmd_vel_pub)

    docking_pub = rospy.Publisher("/rov/docking_done", Bool, queue_size = 10)

    rospy.spin()