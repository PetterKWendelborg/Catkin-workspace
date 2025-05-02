#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import time
from geometry_msgs.msg import Point
# from matplotlib import pyplot as plt 
# import os
# print("Current working directory:", os.getcwd())

mtx = np.array([[1.10904780e+03,   0,         6.38431658e+02],
                [  0,         1.10907114e+03, 3.59179086e+02],
                [  0,           0,           1        ]])

dist = np.array([[ -7.76239086e-04, 6.14575931e-03, -5.54007822e-05, -7.31757780e-05, -1.35769474e-02]])

x_tvecs = []
y_tvecs = []
cv_bridge = CvBridge()

detector = aruco.DetectorParameters_create()

aruco_dict_old = aruco.Dictionary_get(aruco.DICT_5X5_50)
aruco_dict_new = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)

marker_size = 0.174
axis_size = 0.05

#Object points / 3d points
obj_points = np.array([
[-marker_size / 2, marker_size / 2, 0],
[ marker_size / 2, marker_size / 2, 0],
[ marker_size / 2, -marker_size / 2, 0],
[-marker_size / 2, -marker_size / 2, 0]
], dtype=np.float32) 


PnP_flag = cv.SOLVEPNP_ITERATIVE


def image_callback(msg):

    try:
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")  
        # rospy.loginfo("camera source found") 
            
    except Exception as e:
        rospy.logerr(f"cv bridge error {e}")
        return
    
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict_old, parameters= detector)

    if ids is not None:    
        # rospy.loginfo("id found")
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            
            success, rvecs, tvecs = cv.solvePnP(obj_points, corners[i], mtx, dist, flags = PnP_flag)

            cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)
            if success:
            # Predtermined ids/ we had id 0 and id 1
                if ids[i] == 0:

                    if tvecs is not None:
                        rospy.loginfo(f"tvecs: {tvecs[0][0]} {tvecs[1][0]} {tvecs[2][0]}")

                else:
                    rospy.loginfo("id not found")
        
    # Shows each manipulated frames
    cv.imshow("frame",frame)    
    cv.waitKey(1) 
               
if __name__ == "__main__":
    rospy.init_node("view_aruco_marker")
    # aruco_pub = rospy.Publisher("/rov/aruco_center", Point, queue_size=10)
    rospy.Subscriber("/rov/camera/image_raw", Image, image_callback)
    rospy.spin()
    cv.destroyAllWindows()