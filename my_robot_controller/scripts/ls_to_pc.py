#!/usr/bin/env python3

# import sensor_msgs.point_cloud2 as pc2
# import rospy
# from sensor_msgs.msg import PointCloud2, LaserScan
# import laser_geometry.laser_geometry as lg
# import math
# rospy.init_node("ls_to_pc")
# lp = lg.LaserProjection()
# pc2_pub = rospy.Publisher("conv_pc", PointCloud2, queue_size=10)
# def scan(msg):
#     pc2_msg = lp.projectLaser(msg)
#     pc2_msg.header.frame_id = msg.header.frame_id
#     pc2_pub.publish(pc2_msg)
#     point_generator = pc2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)
    # sum = 0.0
    # num = 0
    # for point in point_generator:
    #     if not math.isnan(point[2]):
    #         sum += point[2]
    #         num +=1
    # if num > 0:
    #     print(sum/num)
    # else:
    #     print("No valid points detected")
# rospy.Subscriber("/box/sonar", LaserScan, scan)
# rospy.spin()


# import rospy
# import laser_geometry.laser_geometry as lg
# from sensor_msgs.msg import LaserScan, PointCloud2


# def ls_callback(data):
#     # Convert LaserScan to PointCloud2
#     pc2 = lp.projectLaser(data)
#     pc2.header.frame_id = "depth_camera_link"  # Set the frame ID
#     pub.publish(pc2)

# def ls_to_pointcloud():
#     rospy.init_node('ls_to_pc', anonymous=True)
#     global lp, pub
#     lp = lg.LaserProjection()
#     pub = rospy.Publisher("pc2_conv", PointCloud2, queue_size=10)
#     rospy.Subscriber("/box/sonar", LaserScan, ls_callback)
#     rospy.spin()

# if __name__ == '__main__':
#     ls_to_pointcloud()



# import rospy
# import math
# import numpy as np
# from sensor_msgs.msg import LaserScan, PointCloud2, PointField
# from sensor_msgs import point_cloud2
# from std_msgs.msg import Header

# def ls_callback(data):
#     # Create a list to hold the 3D points
#     points = []

#     # Get the number of horizontal and vertical samples
#     num_horizontal_samples = len(data.ranges)
#     num_vertical_samples = 30  # Match the vertical samples in the Gazebo sensor

#     # Calculate the angular resolution for horizontal and vertical scans
#     horizontal_resolution = (data.angle_max - data.angle_min) / num_horizontal_samples
#     vertical_resolution = (0.26 - (-0.26)) / num_vertical_samples  # 30 samples over 30 degrees

#     # Iterate over each vertical and horizontal sample
#     for v in range(num_vertical_samples):
#         vertical_angle = -0.26 + v * vertical_resolution  # Calculate the vertical angle
#         for h in range(num_horizontal_samples):
#             horizontal_angle = data.angle_min + h * horizontal_resolution  # Calculate the horizontal angle
#             range_value = data.ranges[h]  # Get the range for this sample

#             # Skip invalid ranges
#             if range_value < data.range_min or range_value > data.range_max:
#                 continue

#             # Convert spherical coordinates (range, horizontal angle, vertical angle) to Cartesian coordinates (x, y, z)
#             x = range_value * math.cos(vertical_angle) * math.cos(horizontal_angle)
#             y = range_value * math.cos(vertical_angle) * math.sin(horizontal_angle)
#             z = range_value * math.sin(vertical_angle)

#             # Add the point to the list
#             points.append([x, y, z])

#     # Create a PointCloud2 message
#     header = Header()
#     header.stamp = rospy.Time.now()
#     header.frame_id = "depth_camera_link"  

#     # Define the fields for the PointCloud2 message
#     fields = [
#         PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
#     ]

#     # Create the PointCloud2 message
#     pc2 = point_cloud2.create_cloud(header, fields, points)

#     # Publish the PointCloud2 message
#     pub.publish(pc2)

# import rospy
# import math
# from sensor_msgs.msg import LaserScan, PointCloud2, PointField
# from sensor_msgs import point_cloud2
# from std_msgs.msg import Header

# def ls_callback(data):
#     # Create a list to hold the 3D points
#     points = []

#     # Get the number of horizontal and vertical samples
#     num_horizontal_samples = len(data.ranges)
#     num_vertical_samples = 30  # Match the vertical samples in the Gazebo sensor

#     # Calculate the angular resolution for horizontal and vertical scans
#     horizontal_resolution = (data.angle_max - data.angle_min) / num_horizontal_samples
#     vertical_resolution = (0.26 - (-0.26)) / num_vertical_samples  # 30 samples over 30 degrees

#     # Iterate over each vertical sample
#     for v in range(num_vertical_samples):
#         vertical_angle = -0.26 + v * vertical_resolution  # Calculate the vertical angle

#         # Iterate over each horizontal sample
#         for h in range(num_horizontal_samples):
#             horizontal_angle = data.angle_min + h * horizontal_resolution  # Calculate the horizontal angle
#             range_value = data.ranges[h]  # Get the range for this sample

#             # Skip invalid ranges
#             if range_value < data.range_min or range_value > data.range_max:
#                 continue

#             # Convert spherical coordinates (range, horizontal angle, vertical angle) to Cartesian coordinates (x, y, z)
#             x = range_value * math.cos(vertical_angle) * math.cos(horizontal_angle)
#             y = range_value * math.cos(vertical_angle) * math.sin(horizontal_angle)
#             z = range_value * math.sin(vertical_angle)

#             # Add the point to the list
#             points.append([x, y, z])

#     # Create a PointCloud2 message
#     header = Header()
#     header.stamp = rospy.Time.now()
#     header.frame_id = "depth_camera_link"  

#     # Define the fields for the PointCloud2 message
#     fields = [
#         PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
#     ]

#     # Create the PointCloud2 message
#     pc2 = point_cloud2.create_cloud(header, fields, points)

#     # Publish the PointCloud2 message
#     pub.publish(pc2)

    

# def ls_to_pointcloud():
#     rospy.init_node('ls_to_pc', anonymous=True)
#     global pub
#     pub = rospy.Publisher("pc2_conv", PointCloud2, queue_size=10)
#     rospy.Subscriber("/box/sonar", LaserScan, ls_callback)
#     rospy.spin()

# if __name__ == '__main__':
#     ls_to_pointcloud()



import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import LaserScan
import struct
import math

def scan_callback(scan):
    points = []
    for i, range in enumerate(scan.ranges):
        if range < scan.range_max and range > scan.range_min:
            angle = scan.angle_min + i * scan.angle_increment
            x = range * math.cos(angle)
            y = range * math.sin(angle)
            z = 0  # Assuming 2D sonar
            points.append([x, y, z])

    # Create PointCloud2 message
    cloud = PointCloud2()
    cloud.header.stamp = rospy.Time.now()
    cloud.header.frame_id = "sonar_link"
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = b''.join([struct.pack('fff', x, y, z) for x, y, z in points])

    # Publish the point cloud
    pub.publish(cloud)

if __name__ == '__main__':
    rospy.init_node('sonar_to_pointcloud')
    pub = rospy.Publisher('/sonar_pointcloud', PointCloud2, queue_size=10)
    rospy.Subscriber('/box/sonar', LaserScan, scan_callback)
    rospy.spin()


