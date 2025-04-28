#!/usr/bin/env python3 
import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Wrench

# This code subscribes to the /tms/rov_center topic which gets its information from ponitcloud_to_xyz_3d_tms.py
# here the sonar data is finaly used to adjust the TMS's heading towards the ROV

last_time_in_window = None

def center_call(center_msg, pub):
    global last_time_in_window
    center_x = center_msg.x
    center_z = center_msg.z
    now = rospy.get_time()
    tms_wrench_msg = Wrench()
    boolean = Bool()

    # defining the inner and out hysterisis window in degrees
    inner_hyst_window = 0.5
    outer_hyst_window = 6.0
    # defining the time the TMS must be inside the hysterisis window for heading to be considered complete
    hysteresis_duration = 8
    # defining the thruster force for when outside the inner and outer hysterisis window
    inner_torque = 1
    outer_torque = 4

    if center_z != 0:
        angle_to_center = math.asin(center_x/center_z)
    else:
        angle_to_center = 0

    angle_to_center_gaz_deg = math.degrees(angle_to_center)

    if angle_to_center_gaz_deg >= -inner_hyst_window and angle_to_center_gaz_deg <= inner_hyst_window:
        tms_wrench_msg.torque.z = 0

        # checking if this instance is the first time the TMS is within the inner hysterisis window wihtout having been outside
        if last_time_in_window is None:        
            last_time_in_window = now

        # checking if the TMS has been inside the inner hysterisis window for the desired duration
        elif now - last_time_in_window >= hysteresis_duration:
            # Stayed in window for enough time
            boolean.data = True
            condition_pub.publish(boolean)
            rospy.signal_shutdown("Condition met")

    # if angle is less than -10
    elif angle_to_center_gaz_deg <= -outer_hyst_window:
        tms_wrench_msg.torque.z = outer_torque
        last_time_in_window = None

    # if angle is more than -10 and less than -3
    elif angle_to_center_gaz_deg > -outer_hyst_window and angle_to_center_gaz_deg < -inner_hyst_window :
        tms_wrench_msg.torque.z = inner_torque
        last_time_in_window = None

    # if angle is more than 10
    elif angle_to_center_gaz_deg >= outer_hyst_window:
        tms_wrench_msg.torque.z = -outer_torque
        last_time_in_window = None

    # if angle is less than 10 and more than 3
    elif angle_to_center_gaz_deg < outer_hyst_window and angle_to_center_gaz_deg > inner_hyst_window :
        tms_wrench_msg.torque.z = -inner_torque
        last_time_in_window = None

    else:
        tms_wrench_msg.torque.z = 0


    pub.publish(tms_wrench_msg)


if __name__ == "__main__":
    rospy.init_node("tms_heading")
    
    cmd_vel_pub = rospy.Publisher("/tms/thruster_manager/input", Wrench, queue_size = 10)

    rospy.Subscriber("/tms/rov_center", Point, center_call, cmd_vel_pub)

    condition_pub = rospy.Publisher("/tms/heading_done", Bool, queue_size = 10)

    rospy.spin()