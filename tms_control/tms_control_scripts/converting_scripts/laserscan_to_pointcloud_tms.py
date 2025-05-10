#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg

# This code recieves LaserScan data from the sonar and converts it to PointCloud2 data
# Then it publishes the data to the /tms/sonar/pointcloud topic
# where pointcloud_to_xyz_2d_tms.py subscribes and further uses the data

lp = lg.LaserProjection()

def laserscan_to_pointcloud(msg):
    # Convert LaserScan to PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # Publish the PointCloud2
    pointcloud_pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node("ls_to_pc_tms")
    
    pointcloud_pub = rospy.Publisher("/tms/sonar/pointcloud", PointCloud2, queue_size=10)
    laserscan_sub = rospy.Subscriber("/tms/sonar", LaserScan, laserscan_to_pointcloud)

    rospy.spin()
