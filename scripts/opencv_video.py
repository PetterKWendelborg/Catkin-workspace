#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2




def image_callback(msg):
    cv_bridge = CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Display or process with OpenCV
    cv2.imshow("Camera View", cv_image)
    cv2.waitKey(1)
        
if __name__ == '__main__':
    rospy.init_node('camera_viewer')
    rospy.Subscriber("/rov/camera/image_raw", Image, image_callback)
    rospy.spin()


# def image_callback(msg):
#     bridge = CvBridge()
#     cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#     # Display or process with OpenCV
#     cv2.imshow("Camera View", cv_image)
#     cv2.waitKey(1)
    

# topic_name = "/car/camera/image_raw"
# rate = rospy.Rate(30)
# video_cap_obj = cv2.VideoCapture(0)
# bo = CvBridge()


# if __name__ == '__main__':
#     rospy.init_node('camera_viewer')
#     rospy.Subscriber(topic_name, Image, image_callback)
#     rospy.spin()
#     while not rospy.is_shutdown():
#         returnvalue, capturedFrame

# https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython