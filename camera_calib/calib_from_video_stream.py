#!/usr/bin/env python

import cv2
import numpy as np
import glob
import time

# Defining the dimensions of checkerboard
CHECKERBOARD = (5,8)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 

# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# Start video capture from the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    time.sleep(0.1)
    ret, img = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    print(ret)
    # If desired number of corners are found in the image then ret = true
    if ret == True:
        print('capturing image')
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    
    cv2.imshow('img',img)
    
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

# Now perform the camera calibration based on the captured points
if len(objpoints) > 0 and len(imgpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)
    # print("rvecs : \n")
    # print(rvecs)
    # print("tvecs : \n")
    # print(tvecs)
else:
    print("Calibration was not successful - not enough valid image points captured.")


'''
Right cam: 
[[562.42586682   0.         304.86531274]
 [  0.         559.31246759 270.30609099]
 [  0.           0.           1.        ]]
dist : 

[[-4.22451798e-01  2.86381248e-01 -1.59815170e-04 -2.20573927e-03
  -1.30771400e-01]]

  Back cam:
  Camera matrix : 

[[560.76629998   0.         338.84413723]
 [  0.         562.50572641 248.64097013]
 [  0.           0.           1.        ]]
dist : 

[[-0.40891922  0.2420045   0.00278291 -0.0010987  -0.10939703]]

Left cam:
[[569.04532018   0.         335.16605016]
 [  0.         566.85332382 243.60254152]
 [  0.           0.           1.        ]]
dist : 

[[-4.30647187e-01  2.93329949e-01  5.02598200e-05 -2.25064466e-03
  -1.50939072e-01]]


'''