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

mtx = np.array([[553.81567322,   0,         319.71592178],
                [  0,         554.03405004, 239.90814897],
                [  0,           0,           1        ]])

# dist = np.array([[ 0,  0,  0, 0, 0]])

# mtx = np.array([[11427.3501, 0, 284.011600],
#                 [  0,           11444.4149, 174.604947],
#                 [  0,           0,         1       ]])

dist = np.array([[ -1.12336404e-04, 5.83416985e-03, -3.92632069e-05, 2.03091756e-05, -4.83095205e-03]])



x_tvecs = []
y_tvecs = []
cv_bridge = CvBridge()
marker_size = 0.205
detector = aruco.DetectorParameters_create()

aruco_dict_old = aruco.Dictionary_get(aruco.DICT_5X5_50)

aruco_dict_new = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)

axis_size = 0.05

def image_callback(msg,pub):

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
        # rospy.sleep(2)
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
                    r_mtx, _ = cv.Rodrigues(rvecs) #converts rvecs to a rotation matrix
                    rvecs_inv = r_mtx.T #transpose
                    tvecs_reshaped = tvecs.reshape(3,1)
                    tvecs_inv = -np.dot(rvecs_inv, tvecs_reshaped)
                    # rospy.loginfo(f"tvecs_inv: {tvecs_inv}")
                    # rospy.loginfo(f"R: {np.shape(R)}")
                    # rospy.loginfo(f"R_inv: {np.shape(R_inv)}")
                    # rospy.loginfo(f"tvecs: {tvecs_reshaped}")
                    rospy.loginfo(f"tvecs: {tvecs[0][0][0]} {tvecs[0][0][1]} {tvecs[0][0][2]}")
                    rospy.loginfo(f"tvecs_inv: {tvecs_inv[0][0]} {tvecs_inv[1][0]} {tvecs_inv[2][0]}")
                    # rospy.loginfo(f"rvecs: {rvecs}")

                    #kan kanskje se på Posestaped, men det krever kanskje å se på quaternions
                    center_msg = Point()
                    center_msg.x = tvecs_inv[0][0]
                    center_msg.y = tvecs_inv[1][0]
                    center_msg.z = tvecs_inv[2][0]
                    pub.publish(center_msg)

            else:
                rospy.loginfo("id not found")
        
    # Shows each manipulated frames
    cv.imshow("frame",frame)    
    cv.waitKey(1) 
               
if __name__ == "__main__":
    rospy.init_node("view_aruco_marker")
    aruco_pub = rospy.Publisher("/rov/aruco_5x5", Point, queue_size=10)
    rospy.Subscriber("/rov/camera/image_raw", Image, image_callback, aruco_pub)
    rospy.spin()
    cv.destroyAllWindows()