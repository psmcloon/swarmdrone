#!/usr/bin/env python
# coding: utf-8

# In[1]:


pip install opencv-python


# In[2]:


pip install pupil-apriltags 


# In[3]:


pip install pyyaml


# In[4]:


pip install matplotlib


# In[5]:


import cv2
from pupil_apriltags import Detector
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.cm as cm
import time

cap = cv2.VideoCapture(0) # webcam reference
cap.set(3, 1280)
cap.set(4, 720)

x_vals = []
y_vals = []
z_vals = []

# Camera Calibration Data
# Patrick's Webcam
fxcal = 568.46049015
fycal = 579.62732297
cxcal = 335.88733782
cycal = 243.40490702
tag_size = 0.1075 # meters

# Check if opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

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

while (cap.isOpened()):
    ret, frame = cap.read() # ret = bool for capture success, if success is stored in frame

    gray_img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    tags = at_detector.detect(gray_img, estimate_tag_pose=True, camera_params=(fxcal, fycal, cxcal, cycal), tag_size=tag_size)
    for tag in tags:
        pose_r = tag.pose_R
        pose_t = -tag.pose_t
        homography = tag.homography
        pitch = np.arcsin(-pose_r[2,0]) # pitch dependent on tag orientation
        roll = np.arcsin(pose_r[2,1]/np.cos(pitch)) # roll depedent on tag orientation
        yaw = np.arcsin(pose_r[1,0]/np.cos(pitch))
    
    # print center for troubleshootings
    x_vals.append(float(pose_t[0]))
    y_vals.append(float(pose_t[1]))
    z_vals.append(float(pose_t[2]))
    
    tags = at_detector.detect(gray_img)
    for tag in tags:
#         print(int(tag.center[0]),int(tag.center[1]))
        detected_img = cv2.circle(gray_img, (int(tag.center[0]),int(tag.center[1])), radius=5, color=(200,200,200), thickness=2)
        for corner in tag.corners:
            detected_img = cv2.circle(gray_img, (int(corner[0]), int(corner[1])),radius=5, color=(0,255,0), thickness=2)

#     for troubleshooting display on laptop    
    if type(detected_img) == np.ndarray:
        cv2.imshow("Filtered: Ecs to exit", detected_img)
        save_img = detected_img
    else:
        cv2.imshow("Filtered: Ecs to exit", frame)
        save_img = frame
    detected_img = frame; # reset img definition
    
    c = cv2.waitKey(1) # display popup window
    if c == 27: # until escape is pressed
        break
    
cap.release()
cv2.destroyAllWindows()

# 3d center plot
t = np.arange(len(x_vals)) # color gradient
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(projection='3d')
ax.scatter(x_vals, y_vals, z_vals, c=t, cmap='hsv')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z') 
ax.axes.set_xlim3d(left=-1, right=1) 
ax.axes.set_ylim3d(bottom=-1, top=1) 
ax.axes.set_zlim3d(bottom=0, top=1) 
plt.show()


# In[6]:


from platform import python_version
print(python_version())
# 3.7.16 works for pupil apriltag library


# In[ ]:




