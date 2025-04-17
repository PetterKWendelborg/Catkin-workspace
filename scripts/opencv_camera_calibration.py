#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import glob
import os

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
images = glob.glob(os.path.join(script_dir,'*.png'))
# images_not = glob.glob('*.png')

for fname in images:
    img = cv.imread(fname)
    # img = cv.resize(img, (1280, 720))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
 
 # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)
 
 # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
 
    corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    imgpoints.append(corners)
 
 # Draw and display the corners
    cv.drawChessboardCorners(img, (9,6), corners2, ret)
    cv.imshow('img', img)
    cv.waitKey(500)
 
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, image_size, None, None)
    
print("camera matrix: \n", mtx)
print()

print("distorition coefficients: \n", dist)
print()

# https://docs.opencv.org/4.2.0/dc/dbb/tutorial_py_calibration.html

#Dag_dypt intrinsic verdier

# mtx = np.array([[891.46672299,   0,         706.04060479],
#                 [  0,         887.85519892, 397.87177248],
#                 [  0,           0,         1       ]])

# dist = np.array([[ -0.14980668,  0.06914326,  0.00286741, -0.00554665, -0.06393792]])


print(images)

print()

# print("Current working directory:", os.getcwd())
print(__file__)






