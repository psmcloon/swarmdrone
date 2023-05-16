"""
M E 495B
Spring 2023
Mechatronics Capstone: Swarm Drone Team 1
Michael Hu, Kohya Kato, Patrick McLoon, Finnley Meinig, Emily Nguyen
"""

# import libraries
import time
import RPi.GPIO as GPIO
import numpy as np
GPIO.setmode(GPIO.BCM)
    
def track(): ## add check for collision
    """
    TRACKING PLAN:
    rotate in direction of tag
    move to april tag location
    if obscured - go to collisionavoidance
    """

    #offset = 0

    try:
        while True:                
            # Add lines here to pull rotation matrix as pose_r and translation matrix as pose_t
            pose_r = [[ 0.95254334  0.21532604  0.21516476] [-0.2826669   0.88799531  0.36271718] [-0.11296285 -0.40632379  0.90671957]] # Dummy tag reading until Emily finishes her code
            pose_t = [[-0.02595762] [-0.06662991] [-1.36749298]] # Dummy tag reading until Emily finishes her code

            pitch = np.arcsin(-pose_r[2,0]) # pitch dependent on tag orientation
            roll = np.arcsin(pose_r[2,1]/np.cos(pitch)) # roll depedent on tag orientation
            yaw = np.arcsin(pose_r[1,0]/np.cos(pitch))
            arr1 = np.array(pose_r)
            cpose_r = np.linalg.inv(arr1)
            cpose_t = np.array(-1*pose_t)
            tframepose_t = np.matmul(cpose_r, cpose_t)
            cframepose_t = -tframepose_t # Transfer origin back to drone
            yawarray = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]) # define rotation matrix
            position = np.matmul(np.linalg.inv(yawarray), cframepose_t)

            # x coord: position[0], y coord: position[1], z coord: position[2], yaw: -yaw (check signs on yaw, might be wrong)
            GR_Dist = np.sqrt(position[0]**2 + position[1]**2) #total distance to travel
            GR_HDNG = -yaw #Heading/azimuth angle, clockwise position

            NextState = "track"
            
            #global offset
            Distance threshold = 0 + #offset #Distance threshold, possibly can be increased if drone is stuck
            if position[2] == 0:
                print("Transition to Apriltag Search")
                NextState = "notdetected"
           
            #Goal is to set heading equal to zero - likely through some form of PID control
            while GR_HDNG > 0: #Some consideration could be made for acceptable heading threshold/pid control
                difference = GR_HDNG - 0
                if np.sign(difference) > 0
                    print("Rotate right")
                else
                    print("Rotate left")
                pitch = np.arcsin(-pose_r[2,0]) # pitch dependent on tag orientation
                roll = np.arcsin(pose_r[2,1]/np.cos(pitch)) # roll depedent on tag orientation
                yaw = np.arcsin(pose_r[1,0]/np.cos(pitch))
                GR_HDNG = -yaw #Need to update this part

            while GR_Dist > threshold: 
                front = distance(F)
                if front > threshold:
                    print("Move forward") #may want to set up a waypoint system, i.e saving the dist only at certain points rather than constantly.
                else:
                    print("Transition to collision avvoidance")
                    NextState = "avoid"
                # Add math to calculate movement speed or distance

                pitch = np.arcsin(-pose_r[2,0]) # pitch dependent on tag orientation
                roll = np.arcsin(pose_r[2,1]/np.cos(pitch)) # roll depedent on tag orientation
                yaw = np.arcsin(pose_r[1,0]/np.cos(pitch))
                arr1 = np.array(pose_r)
                cpose_r = np.linalg.inv(arr1)
                cpose_t = np.array(-1*pose_t)
                tframepose_t = np.matmul(cpose_r, cpose_t)
                cframepose_t = -tframepose_t # Transfer origin back to drone
                yawarray = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]) # define rotation matrix
                position = np.matmul(np.linalg.inv(yawarray), cframepose_t)

                GR_Dist = np.sqrt(position[0]**2 + position[1]**2)
                #if stuck: 
                  #offset = offset + 0.1    
                #if GR_Dist <= threshold:
                  #offset = 0 #reset offset

    except KeyboardInterrupt:
        print("Measurement stopped by user")

    return NextState

def avoid():
    """
    COLLISION AVOIDANCE PLAN:
    determine which direction of movement is blocked
    move accordingly
    revert to tracking
    """

    def distance(GPIOpin):
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

    try:
        while True:
            droneLength = 0.26 #meters
            F = 17 #Pin 
            L = 26 #Pin
            R = 6 #Pin
            B = 16 #Pin
            front = distance(F)
            #if front >= droneLength: #will not be necessary in integration, as well as first if statement
            #    print ("move foward") 
            #elif front < droneLength:
            left = distance(L)
            right = distance(R)
            rear = distance(B)
            if left >= droneLength:
                while front <= droneLength:
                    print("rotate left 1 degree") # Will rotate continuously until obstacle is no longer detected in path
                    front = distance(F) # Update while loop condition
            elif right >= droneLength:
                while front <= droneLength:
                    Print("rotate right 1 degree")
                    front = distance(F) 
            elif rear >= dronelength:
                left = distance(L)
                right = distance(R)
                while (left < droneLength and rear >= dronelength) or (right < droneLength and rear >= dronelength): # monitoring sides and rear while reversing
                    print("move backwards")
                    left = distance(L) 
                    right = distance(R) 
                if left > droneLength:
                    front = distance(F)
                    while front < droneLength:
                        print("rotate left 1 degree")
                        front = distance(F)
                elif right > droneLength :
                    while front < droneLength:
                        print("rotate right 1 degree")
            else:
                print("stuck") #global variable stuck = 1
            #Consideration made for changing altitude, experimentation required

            while (left < droneLength or right < droneLength) and front > droneLength: # Move forward to clear obstacle
                front = distance(F) # Used to detect if there are additional obstacles in front
                left = distance(L) # motitor current obstacle
                right = distance(R) # motitor current obstacle
                print("move foward")  
            #Print("Transition to tracking)
    except KeyboardInterrupt:
        print("Measurement stopped by user")
        GPIO.cleanup()

    NextState = "track"
    return NextState

def notdetected():
    """
    TAG NOT DETECTED PLAN:
    wait for 20 seconds, checking for tag once every second.
    If tag is detected, transition to tracking state immediately.
    If tag is not detected for 20 seconds, transition to landing state.

    """

    for x in range(20):
        time.sleep(1)
        position[2] = 0

        # Mavlink command
        print('hold position')

        # search for april tag


        if position[2] != 0:
            NextState = 'track'
            return NextState
        
    print('transition to landing state')
       
track()
while True:
    if NextState == "track":
        track()
    elif NextState == "avoid":
        avoid()
    elif NextState == "notdetected":
        notdetected()
