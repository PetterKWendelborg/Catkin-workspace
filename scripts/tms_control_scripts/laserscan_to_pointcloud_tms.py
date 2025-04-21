#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg

# This code recieves LaserScan data from the sonar and converts it to PointCloud2 data
# Then it publishes the data to the /tms/sonar/pointcloud topic
# where pcdata_conv_tms_2d/3d.py subscribes and further uses the data

lp = lg.LaserProjection()

def scan_cb(msg):
    # Convert LaserScan to PointCloud2
    pc2_msg = lp.projectLaser(msg)
    #rospy.loginfo(pc2_msg)
    # Publish the PointCloud2
    pc_pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node("ls_two_pc_tms")
    
    pc_pub = rospy.Publisher("/tms/sonar/pointcloud", PointCloud2, queue_size=10)
    scan_sub = rospy.Subscriber("/tms/sonar", LaserScan, scan_cb)

    rospy.spin()
