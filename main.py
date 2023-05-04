from statemachine import StateMachine, State
from pymavlink import mavutil
import time

"""
M E 495B
Spring 2023
Mechatronics Capstone: Swarm Drone Team 1
Michael Hu, Kohya Kato, Patrick McLoon, Finnley Meinig, Emily Nguyen
"""

class drone(StateMachine):

    # sequence of events

    def start(self):
        """
        LAUNCH PLAN:
        initialize sensors + camera
        zero flight controller altitude
        rise to designated height
        detect april tag
        if successfully detected, move to tracking
        """

        master = mavutil.mavlink_connection("/dev/ttyACM0", baud=xxx) # Need serial port + baudrate
        master.reboot_autopilot() # reboots ArduSub board

        master = mavutil.mavlink_connection('udpin:0.0.0.0:9000')
        master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (master.target_system, master.target_component))

        """
        Do we need this? idk
        # Send a message for QGC to read out loud
        # Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
        master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                   "QGC will read this".encode())
        """

        # Arming vehicle
        master.mav.command_long_send(master.target_system,
                                     master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0,
                                     1, 0, 0, 0, 0, 0, 0)

        # wait until arming confirmed (can manually check with master.motors_armed())
        print("Waiting for the vehicle to arm")
        master.motors_armed_wait()
        print('Armed!')


    def tracking(self):
        """
        TRACKING PLAN:
        rotate in direction of tag
        move to april tag location with a speed proportional to the distance to the goal
        if obscured - go to collisionavoidance
        """

    def collisionavoidance(self):
        """
        COLLISION AVOIDANCE PLAN:
        determine which direction of movement is blocked
        move accordingly
        revert to tracking
        """

    def landing(self):
        """
        LANDING PLAN:
        check sensors and altitude
        confirm stationary ground robot
        begin vertical descent
        when altitude = 0, ramp down + shutoff motors
        """
