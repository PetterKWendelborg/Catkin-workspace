#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point

import math


def pc_callback(pc_msg, pub):

    rospy.loginfo("call_args triggered")

    # points_list.clear()

    points = pc2.read_points(pc_msg, field_names= ("x","y","z"), skip_nans = True)
    points_list = list(points)

    
    for point in points:
        points_list.append(point)    

    rospy.loginfo(f"number of points detected: {len(points_list)}")  
    #printer ut individuelle points
    # for i, p in enumerate(points_list):
    #     rospy.loginfo(f"point{i+1}: x= {p[0]:.4f} y={p[1]:.4f} z={p[2]:.4f}")

    
    
    rov = 1
    if len(points_list) >= rov:

        #regner ut hele punktet, men ettersom TMS bare skal rotere z aksen, så tror jeg ikke y behøves
        center_x  = sum(p[0] for p in points_list)/len(points_list)
        center_y  = sum(p[1] for p in points_list)/len(points_list)
        center_z  = sum(p[2] for p in points_list)/len(points_list)

        #regner ut vinklen til senter punktet, men skal overføre den til annen node
        angle_to_center = math.asin(center_x/center_z)
        # angle_to_center_ros = math.asin(center_y/center_x)

        rospy.loginfo(f"center point: x= {center_x:.4f} y={center_y:.4f} z={center_z:.4f}")  
        rospy.loginfo(f"angle to center = {angle_to_center} rad")  
        rospy.loginfo(f"angle to center = {math.degrees(angle_to_center)} deg")  

        center_msg = Point()
        center_msg.x = center_x
        center_msg.y = center_y
        center_msg.z = center_z
        pub.publish(center_msg)
        
    else:
        rospy.loginfo("TMS stands still, rov not detected")

    rospy.loginfo(f"-----------------") 

if __name__ == "__main__":
    rospy.init_node("pointcloud_points")


    center_pub = rospy.Publisher("/rov_center", Point, queue_size=10)

    rospy.Subscriber("/tms/depth_camera/points",PointCloud2,pc_callback, center_pub)

    rospy.spin()


    #https://docs.ros.org/en/noetic/api/geometry_msgs/html/index-msg.html

    #https://docs.ros.org/en/noetic/api/sensor_msgs/html/namespacesensor__msgs_1_1point__cloud2.html#a5d969a10974e5f12199d0da5a7f3ec92

    # https://docs.ros.org/en/jade/api/rospy/html/rospy.topics.Subscriber-class.html