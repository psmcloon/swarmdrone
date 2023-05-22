#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from scipy.spatial.transform import Rotation as Rot
import numpy as np
import RPi.GPIO as GPIO
import time

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
 # GPIO.cleanup()
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
    
    
def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber('tag_detections', AprilTagDetectionArray, callback)

	rospy.spin()

    
if __name__ == '__main__':
	listener()
	GPIO.cleanup()
