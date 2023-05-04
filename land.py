from pymavlink import mavutil
import RPi.GPIO as GPIO
import time

"""
LANDING PLAN:
        check sensors and altitude
        confirm stationary ground robot
        begin vertical descent
        when altitude = 0, ramp down + shutoff motors
"""

master = mavutil.mavlink_connection('udpin:0.0.0.0:9000') # DELETE for main script !!!!!!

# Trigger some condition to break the while loop of the distance sensors

# Stop camera capture
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                             0,
                             527, 0, 0, 0, 0, 0, 0) # MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS

# Disarm vehicle
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             0,
                             0, 0, 0, 0, 0, 0, 0)

