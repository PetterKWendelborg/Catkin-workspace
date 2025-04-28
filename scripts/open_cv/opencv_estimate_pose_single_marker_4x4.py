#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

mtx = np.array([[554.31537017,   0,         319.86790386],
    [  0,         554.27617968, 239.29069902],
    [  0,           0,           1        ]])

dist = np.array([[ -1.12336404e-04, 5.83416985e-03, -3.92632069e-05, 2.03091756e-05, -4.83095205e-03]])

last_time_in_window = None
x_tvecs = []
y_tvecs = []
cv_bridge = CvBridge()
marker_size = 0.04207
detector = aruco.DetectorParameters_create()

aruco_dict_old = aruco.Dictionary_get(aruco.DICT_4X4_250)

aruco_dict_new = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

axis_size = 0.05

def image_callback(msg, pub):
    global last_time_in_window 
    counter = 4
    now = rospy.get_time()
    boolean = Bool()
    try:
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")  
        # rospy.loginfo("camera source found") 
            
    except Exception as e:
        rospy.logerr(f"cv bridge error {e}")
        return
    
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict_old, parameters= detector)

    if ids is not None:    
        # rospy.loginfo("id found")
        aruco.drawDetectedMarkers(frame, corners, ids)
        boolean.data = True
        aruco_detected.publish(boolean)

        for i in range(len(ids)):
            # rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
            # cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)
            # Predtermined ids/ we had id 0 and id 1
            if ids[i] == 0:
                rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
                cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)
                rospy.loginfo(f"tvecs_0: {tvecs[0][0][0]} {tvecs[0][0][1]} {tvecs[0][0][2]}")
                aruco_0 = Point()
                aruco_0.x = tvecs[0][0][0]
                aruco_0.y = tvecs[0][0][1]
                aruco_0.z = tvecs[0][0][2]
                pub.publish(aruco_0)

            elif ids[i] == 3:
                rvecs1, tvecs1, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
                cv.drawFrameAxes(frame, mtx, dist, rvecs1, tvecs1, axis_size)
                rospy.loginfo(f"tvecs_1: {tvecs1[0][0][0]} {tvecs1[0][0][1]} {tvecs1[0][0][2]}")
                aruco_1 = Point()
                aruco_1.x = tvecs1[0][0][0]
                aruco_1.y = tvecs1[0][0][1]
                aruco_1.z = tvecs1[0][0][2]
                pub.publish(aruco_1)

            # else:
            #     rospy.loginfo("id not found")

    elif last_time_in_window is None:
        last_time_in_window = now
        rospy.loginfo(f"ids: {ids}")
    
    elif now - last_time_in_window >= counter:
        boolean.data = False
        aruco_detected.publish(boolean)
        rospy.loginfo("aruco not in frame")
        center_msg = Point()
        center_msg.x = 0.0
        center_msg.y = 0.0
        center_msg.z = 0.0
        pub.publish(center_msg)

    # Shows each manipulated frames
    cv.imshow("frame",frame)    
    cv.waitKey(1) 
               
if __name__ == "__main__":
    rospy.init_node("view_aruco_marker")
    aruco_0 = rospy.Publisher("/rov/aruco_4x4", Point, queue_size=10)
    aruco_detected = rospy.Publisher("/rov/aruco_detect_4x4", Bool, queue_size = 10)
    rospy.Subscriber("/rov/camera/image_raw", Image, image_callback, aruco_0)
    rospy.spin()
    cv.destroyAllWindows()