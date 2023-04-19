#!/usr/bin/env python
# coding: utf-8

# In[ ]:


# IMPORT CODE
import cv2
from pupil_apriltags import Detector
import numpy as np

# INITIALIZATION CODE
    
# Camera Calibration Data
# Kohya's Laptop (delete when replaced with raspberry pi calibration data)
fxcal = 938.61258241
fycal = 937.70826823
cxcal = 669.06224346
cycal = 379.28535514
tag_size = .1075
cap = cv2.VideoCapture(0) # webcam reference
cap.set(3, 1280)
cap.set(4, 720)

# Check if opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# Add Raspberry Pi Calibration Data Here
# fxcal = 
# fycal = 
# cxcal = 
# cycal = 
# tag_size = .1075
# cap = cv2.VideoCapture(0)
# cap.set(3, 1280)
# cap.set(4, 720)
    
# initialize detector
at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# EACH TIME DATA NEEDS TO BE ACQUIRED EXECUTE CODE BELOW
ret, frame = cap.read() # ret = True for capture success, if success is stored in frame
frame = cv2.resize(frame, None, fx=1, fy=1, interpolation=cv2.INTER_AREA) # frame is each image from webcam
gray_img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

tags = at_detector.detect(gray_img, estimate_tag_pose=True, camera_params=(fxcal, fycal, cxcal, cycal), tag_size=tag_size)
for tag in tags:
    pose_r = tag.pose_R
    pose_t = -tag.pose_t
    pitch = np.arcsin(-pose_r[2,0]) # pitch dependent on tag orientation
#     roll = np.arcsin(pose_r[2,1]/np.cos(pitch)) # roll depedent on tag orientation
    yaw = np.arcsin(pose_r[1,0]/np.cos(pitch))

    arr1 = np.array(pose_r)
    cpose_r = np.linalg.inv(arr1)
    cpose_t = np.array(-1*pose_t)
    tframepose_t = np.matmul(cpose_r, cpose_t)
    cframepose_t = -tframepose_t
    yawarray = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    position = np.matmul(np.linalg.inv(yawarray), cframepose_t)
    
    
    print('x: ', position[0], 'y: ', position[1], 'z: ', position[2], 'yaw: ', -yaw, end='\r') # Delete
    
# AT THE END OF THE PROGRAM
cap.release()
cv2.destroyAllWindows()

