#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import math

# This code recieves data by subscribing to the /rov/rov/sonar_3d/points topic
# it then processes the PointCloud2 data and converts it into x, y and z coordinates, and calculates their center points
# which are easier to use in the heading script
# the coords are published to /rov/tms_center, and picked up by rov_heading.py and rov_approach.py

def pc_callback(pc_msg, pub):
    points = pc2.read_points(pc_msg, field_names= ("x","y","z"), skip_nans = True)
    points_list = list(points)
    
    for point in points:
        points_list.append(point)    
    
    tms = 1
    if len(points_list) >= tms:
        #regner ut hele punktet
        center_x  = sum(p[0] for p in points_list)/len(points_list)
        center_y  = sum(p[1] for p in points_list)/len(points_list)
        center_z  = sum(p[2] for p in points_list)/len(points_list)

        # angle_to_center = math.asin(center_x/center_z)
        # rospy.loginfo(f"angle to center = {math.degrees(angle_to_center)} deg")  

        center_msg = Point()
        center_msg.x = center_x
        center_msg.y = center_y
        center_msg.z = center_z
        pub.publish(center_msg)
        
    else:
        rospy.loginfo("TMS stands still, rov not detected")

if __name__ == "__main__":
    rospy.init_node("pointcloud_points")

    center_pub = rospy.Publisher("/rov/tms_center", Point, queue_size=10)
    
    rospy.Subscriber("/rov/rov/sonar_3d/points",PointCloud2, pc_callback, center_pub)

    rospy.spin()