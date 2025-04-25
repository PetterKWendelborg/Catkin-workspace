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

mtx = np.array([[891.46672299,   0,         706.04060479],
                [  0,         887.85519892, 397.87177248],
                [  0,           0,         1       ]])

dist = np.array([[ -0.14980668,  0.06914326,  0.00286741, -0.00554665, -0.06393792]])

# mtx = np.array([[11427.3501, 0, 284.011600],
#                 [  0,           11444.4149, 174.604947],
#                 [  0,           0,         1       ]])

# dist = np.array([[ 0.324104931, -441.418319,  0.00254645510,  0.0117633650, -0.502987127]])



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
    
    # Gray scales the video, kanskje vi ikke trenger å grayscale heller.... får bare la den være
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
                        rospy.loginfo(f"rvecs: {rvecs[0][0]} {rvecs[1][0]} {rvecs[2][0]}")
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
                        # rospy.loginfo(f"tvecs: {tvecs[0][0][0]} {tvecs[0][0][1]} {tvecs[0][0][2]}")
                        # rospy.loginfo(f"tvecs_inv: {tvecs_inv[0][0]} {tvecs_inv[1][0]} {tvecs_inv[2][0]}")
                        # rospy.loginfo(f"rvecs: {rvecs}")

                        #kan kanskje se på Posestaped, men det krever kanskje å se på quaternions
                        # center_msg = Point()
                        # center_msg.x = tvecs_inv[0][0]
                        # center_msg.y = tvecs_inv[1][0]
                        # center_msg.z = tvecs_inv[2][0]
                        # pub.publish(center_msg)

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