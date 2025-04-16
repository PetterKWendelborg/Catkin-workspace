#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point

import math

center_y = 1.7054755315184593

center_x = 1.6345236748456955

center = center_y/center_x

# value = math.asin(center)

print(center)
# print(value)



import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from matplotlib import pyplot as plt 



# Selects the video file from files
cap = cv.VideoCapture('ny_dagsvideo.mp4')


#natt_dypt intrinsic values

# mtx = np.array([[839.6394692,   0,         638.77483753],
#                 [  0,         824.87000154, 357.23220596],
#                 [  0,           0,         1       ]])

# dist = np.array([[-0.19215383,  0.17411084,  0.00350001, -0.00104042, -0.15623537]])



#Dag_dypt intrinsic values

mtx = np.array([[891.46672299,   0,         706.04060479],
                [  0,         887.85519892, 397.87177248],
                [  0,           0,         1       ]])

dist = np.array([[ -0.14980668,  0.06914326,  0.00286741, -0.00554665, -0.06393792]])

parameters =  cv.aruco.DetectorParameters()
#7x7
x1_tvecs = []
y1_tvecs = []
x0_tvecs = []
y0_tvecs = []

#5x5
x1_tvecs1 = []
y1_tvecs1 = []
x0_tvecs1 = []
y0_tvecs1 = []

#6x6
x1_tvecs2 = []
y1_tvecs2 = []
x0_tvecs2 = []
y0_tvecs2 = []

#4x4
x1_tvecs3 = []
y1_tvecs3 = []
x0_tvecs3 = []
y0_tvecs3 = []

# Here you can decide the flag input/ PnP method
PnP_flag = cv.SOLVEPNP_IPPE


# #result = cv.VideoWriter('daddy_omid.avi',  
#                          cv.VideoWriter_fourcc(*'MJPG'), 
#                          10, (1280, 720)) 
    

k=0
j=0  
l=0
m=0
t=0
u=0
b=0

while True:
    # Reads the video file
    ret, frame = cap.read()    
    
    # Exit the video if ended 
    if not ret:
        break 
     
    # Resize frames 
    frame = cv.resize(frame, (1280, 720))       
    
    # Gray scales the video
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
      
    # Create the ArUco detector for different bit sizes
    detector = aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_50), parameters)
    detector1 = aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50), parameters)
    detector2 = aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50), parameters)
    detector3 = aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50), parameters)
    # detector = aruco.ArucoDetector(aruco_dic, parameters)

    # Detect the markers/ image points
    corners, ids, rejected = detector.detectMarkers(gray)
    corners1, ids1, rejected1 = detector1.detectMarkers(gray)    
    corners2, ids2, rejected2 = detector2.detectMarkers(gray)
    corners3, ids3, rejected3 = detector3.detectMarkers(gray) 
    
    # Physical size of ArUco tag in meters
    marker_size_big = 0.174  
    marker_size_small = 0.0989
    
    #Object points / 3d points
    obj_points_big = np.array([
    [-marker_size_big / 2, marker_size_big / 2, 0],
    [ marker_size_big / 2, marker_size_big / 2, 0],
    [ marker_size_big / 2, -marker_size_big / 2, 0],
    [-marker_size_big / 2, -marker_size_big / 2, 0]
    ], dtype=np.float32)   

    obj_points_small = np.array([
    [-marker_size_small / 2, marker_size_small / 2, 0],
    [ marker_size_small / 2, marker_size_small  / 2, 0],
    [ marker_size_small / 2, -marker_size_small / 2, 0],
    [-marker_size_small / 2, -marker_size_small  / 2, 0]
    ], dtype=np.float32)   

#-----------------------------------------------------------------------------------------------       
    #7x7
    
    if ids is not None:    

        
        for i in range(len(ids)):


           # Predtermined ids/ we had id 0 and id 1
           if ids[i] == 1:
                # Calculates the extrinsic values
                success, rvecs, tvecs,= cv.solvePnP(obj_points_small, corners[i], mtx, dist, flags = PnP_flag)
                
                # If marker is seen, draw makers, id and and frame axes
                if success:
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, length = 0.1)      
                    
                    # If tvecs is found, then append the x and y translational vectors to a list,
                    # and print the z (distanse from ROV to aruco tag)
                    if tvecs is not None:
                        if ids[i] == 1:
                            k = k+1
                            x1_tvecs.append(tvecs[0][0]) 
                            y1_tvecs.append(tvecs[1][0]) 
                            print("z1_7x7(m): ", tvecs[2][0])
                               
                
           if ids[i] == 0:
                success, rvecs, tvecs,= cv.solvePnP(obj_points_big, corners[i], mtx, dist, flags = PnP_flag)
                
                if success:
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, length = 0.1)  
                       
                    # If tvecs is found, then append the x and y translational vectors to a list,
                    # and print the z (distanse from ROV to aruco tag)
                    if tvecs is not None:

                        if ids[i] == 0: #id is known
                            j= j+1                         
                            x0_tvecs.append(tvecs[0][0]) 
                            y0_tvecs.append(tvecs[1][0]) 
                            print("z0_7x7(m): ", tvecs[2][0])     
  
            
#-----------------------------------------------------------------------------------------------            
    # #5x5
    if ids1 is not None:
        for i in range(len(ids1)):

            if ids1[i] == 1:
                 success1, rvecs1, tvecs1,= cv.solvePnP(obj_points_small, corners1[i], mtx, dist, flags = PnP_flag)
                 
                 if success1:
                     aruco.drawDetectedMarkers(frame, corners1, ids1)
                     cv.drawFrameAxes(frame, mtx, dist, rvecs1, tvecs1, length = 0.1)                  
             
                     #plots for cv.solvePnP
                     if tvecs1 is not None:
                         if ids1[i] == 1:
                             x1_tvecs1.append(tvecs1[0][0]) 
                             y1_tvecs1.append(tvecs1[1][0]) 
                             # print("z1_5x5(m): ", tvecs1[2][0])
                    
            if ids1[i] == 0:
                 success1, rvecs1, tvecs1,= cv.solvePnP(obj_points_big, corners1[i], mtx, dist, flags = PnP_flag)
                 
                 if success1:
                     aruco.drawDetectedMarkers(frame, corners1, ids1)
                     cv.drawFrameAxes(frame, mtx, dist, rvecs1, tvecs1, length = 0.1)                  
  
                     if tvecs1 is not None:
                        if ids1[i] == 0:
                             x0_tvecs1.append(tvecs1[0][0]) 
                             y0_tvecs1.append(tvecs1[1][0])  
                             # print("z0_5x5(m): ", tvecs1[2][0])
    
    
    # result.write(frame)     
    
    # Shows each manipulated frames
    cv.imshow('frame',frame)
    
    # If "q" is pressed, end the video
    if cv.waitKey(1) & 0xFF == ord("q"):
        break
    
# Release the video and destroy the video windows
cap.release()
#result.release()
cv.destroyAllWindows()



fig, ax = plt.subplots(figsize= (8,6))

ax.plot(x1_tvecs, y1_tvecs, color = "blue", marker = "." , label = "Marker ID 1 (7x7)")
ax.plot(x0_tvecs, y0_tvecs, color = "red", marker = "." , label = "Marker ID 0 (7x7)")

# ax.plot(x1_tvecs1, y1_tvecs1, color = "pink", marker = "." , label = "Marker ID 1 (5x5)")
# ax.plot(x0_tvecs1, y0_tvecs1, color = "brown", marker = "." , label = "Marker ID 0 (5x5)")


ax.set_title("translational vectors in x and y direction (IPPE)")
ax.set_ylabel("Y (m)")
ax.set_xlabel("X (m)")
ax.grid()
ax.legend(loc = "upper left")
print()
# print("id1_7x7:",k ,"id0_7x7:",j)
# print("id1_6x6:",l ,"id0_6x6:",m)
# print("id1_4x4:",t ,"id0_4x4:",u)

