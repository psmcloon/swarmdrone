#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from scipy.spatial.transform import Rotation as Rot
import numpy as np
import RPi.GPIO as GPIO
import time
import cv2
import matplotlib.pyplot as plt

# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#offset = 0
#global offset offset may not required if stuck just transitions to landing
distthresh = 0.1 #+ offset #Distance threshold, possibly can be increased if drone is stuck, remove offset
hdngthresh = 0.1
droneLength = 0.1 #meters
F = 17 #Pin 
L = 26 #Pin
R = 6 #Pin
B = 16 #Pin
sleepdur = 0.05
notdetect = 0
NextState = "track"

def getpose(data):
	translation = [[]]
	rotation = [[]]
	
	Dist = []
	HDNG = []
	position = [[]]

	for detection in data.detections:

		posx = detection.pose.pose.pose.position.x
		posy = detection.pose.pose.pose.position.y
		posz = detection.pose.pose.pose.position.z

		quater_rotx = detection.pose.pose.pose.orientation.x
		quater_roty = detection.pose.pose.pose.orientation.y
		quater_rotz = detection.pose.pose.pose.orientation.z
		quater_rotw = detection.pose.pose.pose.orientation.w

		quatermatrix = [quater_rotx, quater_roty, quater_rotz, quater_rotw]

		rotation = Rot.from_quat(quatermatrix)
		rotation = rotation.as_matrix()

		translation = [posx, posy, posz]

		pose_r = np.array(rotation)
		pose_t = np.array(translation)
		
		pitch = np.arcsin(-pose_r[2,0]) # pitch dependent on tag orientation #Note: write as a function instead
		roll = np.arcsin(pose_r[2,1]/np.cos(pitch)) # roll depedent on tag orientation
		yaw = np.arcsin(pose_r[1,0]/np.cos(pitch))
		arr1 = np.array(pose_r)
		cpose_r = np.linalg.inv(arr1)
		cpose_t = np.array(-1*pose_t)
		tframepose_t = np.matmul(cpose_r, cpose_t)
		cframepose_t = -tframepose_t # Transfer origin back to drone
		yawarray = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]) # define rotation matrix
		position = np.matmul(np.linalg.inv(yawarray), cframepose_t)
		
		#x coord: position[0], y coord: position[1], z coord: position[2], yaw: -yaw (check signs on yaw, might be wrong)
		
		#print("x: ", np.round(position[0],2), "y: ", np.round(position[1],2), "z: ", np.round(position[2],2), "yaw: ", np.round(-yaw,2))
		
		Dist = np.sqrt(position[0]**2 + position[1]**2) #total distance to travel
		#GR_HDNG = -yaw*180/np.pi #Heading/azimuth angle, clockwise position
		HDNG = np.arctan2(-position[1],position[0])
	return Dist, HDNG, position


def distance(GPIOpin):#Sort distance according to GPIOpin, labeled as FOWARD, LEFT, RIGHT, and REAR in pseudocode 
    GPIO_SIG = GPIOpin;
    GPIO.setup(GPIO_SIG, GPIO.OUT)
    GPIO.output(GPIO_SIG, 0)

    time.sleep(0.000002)

    #send trigger signal
    GPIO.output(GPIO_SIG, 1)

    time.sleep(0.000005)

    GPIO.output(GPIO_SIG, 0)

    GPIO.setup(GPIO_SIG, GPIO.IN)

    while GPIO.input(GPIO_SIG) == 0:
        starttime = time.time()

    while GPIO.input(GPIO_SIG) == 1:
        endtime = time.time()

    duration = endtime - starttime
    # Distance is defined as time/2 (there and back) * speed of sound 34000 cm/s 
    distance = (duration*34000)/2/100 #return in meters
 
    return distance


def track(GR_Dist, GR_HDNG, position):
  """
  TRACKING PLAN:
  rotate in direction of tag
  move to april tag location
  if obscured - go to collisionavoidance
  """
  notdetect = 0
  NextState = "track"
  #print("dist: ", GR_Dist, "hdg: ", GR_HDNG)
  tol = 0.05
  if position[2] == 0:
    print("Transition to Apriltag Search")
    NextState = "notdetected"
    return NextState
  elif distance(F) <= 0.75*droneLength:
    print("Transition to Collision Avoidance")
    NextState = "avoid"
    return NextState
     
  #This current iteration does not account for the existence of the collision avoidance model.
  #Goal is to set heading equal to zero - likely through some form of PID control
  #while np.absolute(GR_HDNG) > 0.5: #Some consideration could be made for acceptable heading threshold/pid control
      #GR_Dist, GR_HDNG, position = getpose(data)
  if GR_HDNG > hdngthresh:
    print("Rotate right ", GR_HDNG)
  elif GR_HDNG < -hdngthresh:
    print("Rotate left ", GR_HDNG)

  if np.absolute(GR_HDNG) <= hdngthresh:
    if GR_Dist > distthresh+tol and position[0] > 0: #Does not account for existence of collision avoidance script yet. Should transition to collision avoidance if an object is detected
      print("Move forward") #may want to set up a waypoint system, i.e saving the dist only at certain points rather than constantly. 
    elif GR_Dist < distthresh-tol or position[0] < 0:
      print("Move backward")
  return NextState


def avoid():
  """
  COLLISION AVOIDANCE PLAN:
  determine which direction of movement is blocked
  move accordingly
  revert to tracking
  """

  NextState = "avoid"
  front = distance(F) 
  left = distance(L)
  right = distance(R)
  rear = distance(B)
  if left >= droneLength:
    while front <= droneLength:
      time.sleep(sleepdur)
      print("rotate left 1 degree") # Will rotate continuously until obstacle is no longer detected in path
      front = distance(F) # Update while loop condition
  elif right >= droneLength:
    while front <= droneLength:
      time.sleep(sleepdur)
      print("rotate right 1 degree")
      front = distance(F) 
  elif rear >= droneLength:
    left = distance(L)
    right = distance(R)
    while left < droneLength and rear >= droneLength and right < droneLength: # monitoring sides and rear while reversing
      time.sleep(sleepdur)                
      print("move backwards")
      left = distance(L) 
      right = distance(R) 
      rear = distance(B)
    if left > droneLength:
      while front < droneLength:
        time.sleep(sleepdur)
        print("rotate left 1 degree")
        front = distance(F)
    elif right > droneLength:
      while front < droneLength:
        time.sleep(sleepdur)
        print("rotate right 1 degree")
        front = distance(F)
  else:
    print("stuck, transition to landing")
  #Consideration made for changing altitude, experimentation required

  while (left < droneLength or right < droneLength) and front > droneLength: # Move forward to clear obstacle
    time.sleep(sleepdur)            
    front = distance(F) # Used to detect if there are additional obstacles in front
    left = distance(L) # monitor current obstacle
    right = distance(R) # monitor current obstacle
    print("move foward") 
    print("Transition to tracking")
    NextState = "track"
  return NextState

    
def notdetected():
  """
  TAG NOT DETECTED PLAN:
  wait for 20 seconds, checking for tag once every second.
  If tag is detected, transition to tracking state immediately.
  If tag is not detected for 20 seconds, transition to landing state.
  """

  if notdetect < 40:
    notdetect = notdetect + 1
    time.sleep(0.5)

    # Mavlink command
    print('hold position')

    if position[2] != 0:
      NextState = "track"
      return NextState

    NextState = "notdetected"
    return NextState    

  else:
    print('transition to landing state')
    
def callback(data):
  global NextState

    
  GR_Dist, GR_HDNG, position = getpose(data)

  if NextState == "track":
    NextState = track(GR_Dist, GR_HDNG, position)
  elif NextState == "avoid":
    NextState = avoid()
  elif NextState == "notdetected":
    NextState = notdetected()
    
  ### start of display code in callback
  # create blank
  textbar = int(0.2*(0.9*RGB_top.shape[0]));
  window = int(2.5*RGB_top.shape[0])
  blank = 255 * np.ones(shape=[window+3*textbar+1, 2*window+1, 3], dtype=np.uint8)
  plt.axis('off') # turn of axes for design
  # colors
  blocked = (240,80,90) # red
  free = (120,230,100) # green

  # color edits
  if right < trigger_dist: color_right = blocked
  else: color_right = free
  if left < trigger_dist: color_left = blocked
  else: color_left = free
  if rear < trigger_dist: color_rear = blocked
  else: color_rear = free  
  if front < trigger_dist: color_front = blocked
  else: color_front = free
  if top < trigger_dist: color_top = blocked
  else: color_top = free  
  if bottom < trigger_dist: color_bottom = blocked
  else: color_bottom = free
	
  # reference points
  top_center = (int(0.5*window),int(0.5*window))
  top_left = int(top_center[0]-0.5*RGB_top.shape[0])
  top_right = int(top_center[0]+0.5*RGB_top.shape[0])
  top_top = int(top_center[1]-0.5*RGB_top.shape[1])
  top_bottom = int(top_center[1]+0.5*RGB_top.shape[1])
  front_center = (int(0.5*window-0.5*RGB_front.shape[0]),int(1.5*window)) # format (y,x) change to fit format
  front_left = int(front_center[0]-0.5*RGB_front.shape[0])
  front_right = int(front_center[0]+0.5*RGB_front.shape[0])
  front_top = int(front_center[1]-0.5*RGB_front.shape[1])
  front_bottom = int(front_center[1]+0.5*RGB_front.shape[1])

  # colored flags
  rnge = (int(0.9*RGB_top.shape[0]),int(0.9*RGB_top.shape[0]))
  edge = int(0.05*RGB_top.shape[0])
  front_center = (front_center[1],front_center[0])
  align_offset = int(0.5*(RGB_top.shape[0]-RGB_front.shape[0]))

  # colored wedges and white rectangle top view
  cv2.ellipse(blank,top_center,rnge,0,-20,20,color_right,-1)
  cv2.ellipse(blank,top_center,rnge,90,-20,20,color_rear,-1)
  cv2.ellipse(blank,top_center,rnge,180,-20,20,color_left,-1)
  cv2.ellipse(blank,top_center,rnge,270,-20,20,color_front,-1)
  cv2.rectangle(blank,(top_left-edge,top_top-edge),(top_right+edge,top_bottom+edge),(255,255,255),-1)
  # colored wedges and white rectangle rear view
  cv2.ellipse(blank,front_center,rnge,0,-20,20,color_right,-1)
  cv2.ellipse(blank,(front_center[0],front_center[1]-align_offset),rnge,90,-20,20,color_bottom,-1)
  cv2.ellipse(blank,front_center,rnge,180,-20,20,color_left,-1)
  cv2.ellipse(blank,(front_center[0],front_center[1]+align_offset),rnge,270,-20,20,color_top,-1)
  cv2.rectangle(blank,(front_top-edge,front_left-edge),(front_bottom+edge,front_right+edge),(255,255,255),-1)
	
  # insert top down drone image
  blank[top_left:top_right, top_top:top_bottom,:] = RGB_top[0:RGB_top.shape[0],0:RGB_top.shape[1],:]
  # insert front view drone image
  blank[front_left:front_right, front_top:front_bottom,:] = RGB_front[0:RGB_front.shape[0],0:RGB_front.shape[1],:]

  # boarder rectangles
  cv2.rectangle(blank,(2,2),(window,window),(0,0,0),thickness = 1)
  cv2.rectangle(blank,(window,2),(2*window,window),(0,0,0),thickness = 1)
  cv2.rectangle(blank,(2,window),(2*window,window+textbar),(0,0,0),thickness = 1)
  cv2.rectangle(blank,(2,window+textbar),(2*window,window+2*textbar),(0,0,0),thickness = 1)
  cv2.rectangle(blank,(2,window+2*textbar),(2*window,window+3*textbar),(0,0,0),thickness = 1)
	
  # command tree connections, may need help navigating functions
  ### CODE IS INCOMPLETE NEED TO ADD CONNECTIONS HERE

  # explanatory text
  text_edge = int(0.05*RGB_top.shape[0])
  plt.text(text_edge, text_edge, 'Top View', horizontalalignment='left',verticalalignment='top',size = 7)
  plt.text(window+text_edge, text_edge, 'Rear View', horizontalalignment='left',verticalalignment='top',size = 7)
  plt.text(text_edge, int(window+0.5*textbar), "Command: "+command, horizontalalignment='left',verticalalignment='center',size = 7)
  plt.text(text_edge, int(window+1.5*textbar), "AprilTag Coordinates: "+"("+str(tag_x)+" mm, "+str(tag_y)+" mm, "+str(tag_z)+" mm)", horizontalalignment='left',verticalalignment='center',size = 7)
  plt.text(text_edge, int(window+2.5*textbar), "State: "+state, horizontalalignment='left',verticalalignment='center',size = 7)

  # direction label top view
  plt.text(top_center[0], top_center[1] - (rnge[0]+5), 'Front', horizontalalignment='center',verticalalignment='bottom',size = 5,color=[0.3,0.3,0.3])
  plt.text(top_center[0], top_center[1] + (rnge[0]+5), 'Rear', horizontalalignment='center',verticalalignment='top',size = 5,color=[0.3,0.3,0.3])
  plt.text(top_center[0] - (rnge[0]+5), top_center[1], 'Left', horizontalalignment='right',verticalalignment='center',size = 5,color=[0.3,0.3,0.3])
  plt.text(top_center[0] + (rnge[0]+5), top_center[1], 'Right', horizontalalignment='left',verticalalignment='center',size = 5,color=[0.3,0.3,0.3])

  # friction label front view
  plt.text(front_center[0],front_center[1]-(rnge[0]+5)+int(0.5*(RGB_top.shape[0]-RGB_front.shape[0])), 'Top', horizontalalignment='center',verticalalignment='bottom',size = 5,color=[0.3,0.3,0.3])
  plt.text(front_center[0],front_center[1]+(rnge[0]+5)-int(0.5*(RGB_top.shape[0]-RGB_front.shape[0])), 'Bottom', horizontalalignment='center',verticalalignment='top',size = 5,color=[0.3,0.3,0.3])
  plt.text(front_center[0] - (rnge[0]+5), front_center[1], 'Left', horizontalalignment='right',verticalalignment='center',size = 5,color=[0.3,0.3,0.3])
  plt.text(front_center[0] + (rnge[0]+5), front_center[1], 'Right', horizontalalignment='left',verticalalignment='center',size = 5,color=[0.3,0.3,0.3])

  # distance reading here
  # flag distance top text
  plt.text(top_center[0], top_center[1] - (rnge[0]+5) + 53, read_front, horizontalalignment='center',verticalalignment='bottom',size = 7)
  plt.text(top_center[0], top_center[1] + (rnge[0]+5) - 53, read_rear, horizontalalignment='center',verticalalignment='top',size = 7)
  plt.text(top_center[0] - (rnge[0]+5) + 59, top_center[1], read_left, horizontalalignment='right',verticalalignment='center',size = 7)
  plt.text(top_center[0] + (rnge[0]+5) - 59, top_center[1], read_right, horizontalalignment='left',verticalalignment='center',size = 7)

  # flag distance front text
  plt.text(front_center[0],front_center[1]-(rnge[0]+5)+56+50, read_top, horizontalalignment='center',verticalalignment='bottom',size = 7)
  plt.text(front_center[0],front_center[1]+(rnge[0]+5)-56-50, read_bottom, horizontalalignment='center',verticalalignment='top',size = 7)
  plt.text(front_center[0] - (rnge[0]+5)+59, front_center[1], read_left, horizontalalignment='right',verticalalignment='center',size = 7)
  plt.text(front_center[0] + (rnge[0]+5)-59, front_center[1], read_right, horizontalalignment='left',verticalalignment='center',size = 7)

  # tag bound management
  if position[0] <= 1000 and position[0] >= -1000:
      top_fr = position[0]/5
  elif position[0] > 1000:
      top_fr = 1000
  elif position[0] < -1000:
      top_fr = -1000
  else:
      top_fr = 0
    
  if position[1] <= 1000 and position[1] >= -1000:
      top_rl = position[1]/5
  elif position[1] > 1000:
      top_rl = 1000
  elif position[1] < -1000:
      top_rl = -1000
  else:
      top_rl = 0

  if position[2] >= -1200 and position[2] <= 0:
      front_tb = position[2]/5
  elif position[2] < -1200:
      front_tb = -1200
  else:
      front_tb = 0

  plt.plot(top_rl+top_center[0], -top_fr+top_center[1], marker="s", markersize=15, markeredgecolor="white",markeredgewidth=1.5, markerfacecolor=[1,0.9,0])
  plt.plot(top_rl+front_center[0], -front_tb+front_center[1], marker="s", markersize=15, markeredgecolor="white", markeredgewidth=1.5, markerfacecolor=[1,0.9,0])
  plt.text(top_rl+top_center[0], -top_fr+top_center[1], "Tag", horizontalalignment='center',verticalalignment='center',size = 7,color="black")
  plt.text(top_rl+front_center[0], -front_tb+front_center[1], "Tag", horizontalalignment='center',verticalalignment='center',size = 7,color="black")

  plt.imshow(blank)
  plt.show()
### end of display code in callback
    
def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber('tag_detections', AprilTagDetectionArray, callback)

	rospy.spin()

    
if __name__ == '__main__':
	listener()
	GPIO.cleanup()
