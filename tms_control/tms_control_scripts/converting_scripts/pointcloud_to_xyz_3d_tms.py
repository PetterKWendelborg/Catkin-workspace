#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, PointStamped
import math

# This code recieves data by subscribing to the /tms/tms/sonar_3d/points topic
# it then processes the PointCloud2 data and converts it into x, y and z coordinates, and calculates their center points
# which are easier to use in the heading script
# the coords are published to /tms/rov_center, and picked up by tms_heading_3d.py

def pc_callback(pc_msg, pub):
    global angle_to_center
    points = pc2.read_points(pc_msg, field_names= ("x","y","z"), skip_nans = True)
    points_list = list(points)
    
    for point in points:
        points_list.append(point)    

    rov = 1
    if len(points_list) >= rov:
        #regner ut hele punktet
        center_x  = sum(p[0] for p in points_list)/len(points_list)
        center_y  = sum(p[1] for p in points_list)/len(points_list)
        center_z  = sum(p[2] for p in points_list)/len(points_list)

        if center_z != 0:
            angle_to_center = math.asin(center_x/center_z)
            # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center)} deg")  
        else:
            angle_to_center = 0

        # center_msg = Point()
        # center_msg.x = center_x
        # center_msg.y = center_y
        # center_msg.z = center_z
        # pub.publish(center_msg)

        center_msg = PointStamped()
        center_msg.header.stamp =  pc_msg.header.stamp
        center_msg.header.frame_id= "/tms/rov_center"
        center_msg.point.x = center_x
        center_msg.point.y = center_y
        center_msg.point.z = center_z
        pub.publish(center_msg)
        
        # publishes the angle as the value z and the time it is published
        angle_to_center_msg = PointStamped()
        angle_to_center_msg.header.stamp =  pc_msg.header.stamp
        angle_to_center_msg.header.frame_id= "/tms/rov_center_angle"
        angle_to_center_msg.point.z = math.degrees(angle_to_center)
        # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center)} deg")  

        rov_center_angle.publish(angle_to_center_msg)  

    # else:
        # rospy.loginfo("TMS stands still, rov not detected")

if __name__ == "__main__":
    rospy.init_node("pointcloud_points")

    center_pub = rospy.Publisher("/tms/rov_center", PointStamped, queue_size=10)

    rov_center_angle = rospy.Publisher("/tms/rov_center_angle", PointStamped, queue_size=10)

    rospy.Subscriber("/tms/tms/sonar_3d/points",PointCloud2,pc_callback, center_pub)
    rospy.Subscriber("/tms/sonar/pointcloud",PointCloud2, pc_callback, center_pub)

    rospy.spin()