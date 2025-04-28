#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
# from matplotlib import pyplot as plt 

last_time_in_window = None

mtx = np.array([[553.81567322,   0,         319.71592178],
                [  0,         554.03405004, 239.90814897],
                [  0,           0,           1        ]])

dist = np.array([[ -1.12336404e-04, 5.83416985e-03, -3.92632069e-05, 2.03091756e-05, -4.83095205e-03]])


x_tvecs = []
y_tvecs = []
cv_bridge = CvBridge()
marker_size = 0.42
detector = aruco.DetectorParameters_create()

aruco_dict_old = aruco.Dictionary_get(aruco.DICT_5X5_50)

aruco_dict_new = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)

axis_size = 0.05

def image_callback(msg,pub):
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
    
    # Gray scales the video, kanskje vi ikke trenger å grayscale heller.... får bare la den være
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # detectMarkers er tydelighvis opencv 4.7 og opp
    # corners, ids, rejected = detector.detectMarkers(gray)

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict_old, parameters= detector)

    if ids is not None:    
        rospy.loginfo("id found")
        last_time_in_window = None
        aruco.drawDetectedMarkers(frame, corners, ids)
        boolean.data = True
        aruco_detected.publish(boolean)

        for i in range(len(ids)):
            # rospy.loginfo("inside_loop")
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)

            cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)

            # Predtermined ids/ we have id 0
            if ids[i] == 0:
            #     rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], marker_size, mtx, dist)
            #     cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_size)
                # if tvecs is not None:
                rospy.loginfo(f"tvecs: {tvecs[0][0][0]} {tvecs[0][0][1]} {tvecs[0][0][2]}")

                #kan kanskje se på Posestaped, men det krever kanskje å se på quaternions
                center_msg = Point()
                center_msg.x = tvecs[0][0][0]
                center_msg.y = tvecs[0][0][1]
                center_msg.z = tvecs[0][0][2]
                pub.publish(center_msg)

            else:
                rospy.loginfo("id not found")
    
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


    # rospy.loginfo("no frame")     
if __name__ == "__main__":
    rospy.init_node("view_aruco_marker")
    aruco_pub = rospy.Publisher("/rov/aruco_5x5", Point, queue_size=10)
    aruco_detected = rospy.Publisher("/rov/aruco_detect_5x5", Bool, queue_size = 10)
    rospy.Subscriber("/rov/camera/image_raw", Image, image_callback, aruco_pub)
    rospy.spin()
    cv.destroyAllWindows()