#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import time
# from matplotlib import pyplot as plt 
# import os
# print("Current working directory:", os.getcwd())

mtx = np.array([[891.46672299,   0,         706.04060479],
                [  0,         887.85519892, 397.87177248],
                [  0,           0,         1       ]])

dist = np.array([[ -0.14980668,  0.06914326,  0.00286741, -0.00554665, -0.06393792]])

x_tvecs = []
y_tvecs = []
cv_bridge = CvBridge()
marker_size = 0.174    
detector = aruco.DetectorParameters_create()

aruco_dict_old = aruco.Dictionary_get(aruco.DICT_5X5_50)

aruco_dict_new = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)

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

    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict_old, parameters= detector)

    if ids is not None:    
        rospy.loginfo("id found")
        # rospy.sleep(2)
        aruco.drawDetectedMarkers(frame, corners, ids)


        for i in range(len(ids)):
            
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)

            cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)

            # Predtermined ids/ we had id 0 and id 1
            # if ids[i] == 1:

            #     rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
                
            #     cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)

            #     if tvecs is not None:
            #         if ids[i] == 1:
            #             x_tvecs.append(tvecs[0][0][0]) 
            #             y_tvecs.append(tvecs[0][0][1]) 
            #             print("z_5x5(m): ", tvecs[0][0][2])
        
    else:
        rospy.loginfo("id not found")
        
    # Shows each manipulated frames
    cv.imshow("frame",frame)    
    cv.waitKey(1) 
               
if __name__ == "__main__":
    rospy.init_node("view_aruco_marker")
    rospy.Subscriber("/rov/camera/image_raw", Image, image_callback)
    rospy.spin()
    cv.destroyAllWindows()