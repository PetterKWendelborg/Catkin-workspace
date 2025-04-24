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

# mtx = np.array([[541.81250952,   0,         313.00123798],
#     [  0,         541.50614797, 246.0726591],
#     [  0,           0,           1        ]])

# dist = np.array([[ 0.03586097, -0.38164328,  0.00195863, -0.00596067,  0.53908965]])

mtx = np.array([[554.31537017,   0,         319.86790386],
    [  0,         554.27617968, 239.29069902],
    [  0,           0,           1        ]])

dist = np.array([[ 7.87210398e-04, -2.19484959e-04,  -4.72164260e-05, 1.35818538e-04,  4.61291537e-03]])

x_tvecs = []
y_tvecs = []
cv_bridge = CvBridge()
marker_size = 0.174    
detector = aruco.DetectorParameters_create()

aruco_dict_old = aruco.Dictionary_get(aruco.DICT_4X4_250)

aruco_dict_new = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

axis_size = 0.05

def image_callback(msg):

    try:
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")  
        # rospy.loginfo("camera source found") 
            
    except Exception as e:
        rospy.logerr(f"cv bridge error {e}")
        return
    
    # Gray scales the video, kanskje vi ikke trenger å grayscale heller.... får bare la den være
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    


    # detectMarkers er tydelighvis opencv 4.7 og opp
    # corners, ids, rejected = detector.detectMarkers(gray)

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict_old, parameters= detector)

    if ids is not None:    
        # rospy.loginfo("id found")
        aruco.drawDetectedMarkers(frame, corners, ids)


        for i in range(len(ids)):
            
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)

            cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)

            # Predtermined ids/ we had id 0 and id 1
            if ids[i] == 0:
            #     rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
            #     cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)
                if tvecs is not None:
                    # x_tvecs.append(tvecs[0][0][0]) 
                    # y_tvecs.append(tvecs[0][0][1]) 
                    # r_mtx, _ = cv.Rodrigues(rvecs) #converts rvecs to a rotation matrix
                    # rvecs_inv = r_mtx.T #transpose
                    # tvecs_reshaped = tvecs.reshape(3,1)
                    # tvecs_inv = -np.dot(rvecs_inv, tvecs_reshaped)
                    # rospy.loginfo(f"tvecs_inv: {tvecs_inv}")
                    # rospy.loginfo(f"R: {np.shape(R)}")
                    # rospy.loginfo(f"R_inv: {np.shape(R_inv)}")
                    # rospy.loginfo(f"tvecs: {tvecs_reshaped}")
                    rospy.loginfo(f"tvecs: {tvecs[0][0][0]} {tvecs[0][0][1]} {tvecs[0][0][2]}")
                    # rospy.loginfo(f"tvecs_inv: {tvecs_inv[0][0]} {tvecs_inv[1][0]} {tvecs_inv[2][0]}")
                    # rospy.loginfo(f"rvecs: {rvecs}")
            elif ids[i] == 1:
            #     rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
            #     cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)
                if tvecs is not None:

                    rospy.loginfo(f"tvecs: {tvecs[0][0][0]} {tvecs[0][0][1]} {tvecs[0][0][2]}")

            elif ids[i] == 2:
            #     rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
            #     cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)
                if tvecs is not None:

                    rospy.loginfo(f"tvecs: {tvecs[0][0][0]} {tvecs[0][0][1]} {tvecs[0][0][2]}")

            elif ids[i] == 3:
            #     rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
            #     cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)
                if tvecs is not None:

                    rospy.loginfo(f"tvecs: {tvecs[0][0][0]} {tvecs[0][0][1]} {tvecs[0][0][2]}")
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