#!/usr/bin/env python3 
import rospy
import math
from geometry_msgs.msg import Point, Wrench
from std_msgs.msg import Bool

rov_approach_done = False
last_time_in_window = None

def start_docking(aruco_msg, pub):
    # global condition
    distance_rov_to_aruco_5x5 = aruco_msg.z

    stopping_distance = 0.05
    rov_wrench_msg = Wrench()
    boolean = Bool()
    global last_time_in_window
    global rov_approach_done

    if not rov_approach_done:

        if distance_rov_to_aruco_5x5 <= stopping_distance:
            rov_wrench_msg.force.x = 0
            pub.publish(rov_wrench_msg)
            boolean.data = True
            docking_pub.publish(boolean)

        elif distance_rov_to_aruco_5x5 > stopping_distance:
            rov_wrench_msg.force.x = 2

        else:
            rov_wrench_msg.force.x = 0

        pub.publish(rov_wrench_msg)
            
    else:
        rospy.loginfo(f"condition is {rov_approach_done}")
    

def condition_call(docking_ready):
    global rov_approach_done
    if docking_ready.data:
        rov_approach_done = docking_ready.data

if __name__ == "__main__":
    rospy.init_node("rov_docking")

    cmd_vel_pub = rospy.Publisher("/rov/thruster_manager/input", Wrench, queue_size = 10)
    
    rospy.Subscriber("/rov/approach_done", Bool, condition_call)

    rospy.Subscriber("/rov/aruco_5x5", Point, start_docking, cmd_vel_pub)

    docking_pub = rospy.Publisher("/rov/docking_done", Bool, queue_size = 10)

    rospy.spin()