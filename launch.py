from pymavlink import mavutil
import RPi.GPIO as GPIO
import time

        """
        LAUNCH PLAN:
        initialize sensors + camera
        zero flight controller altitude
        rise to designated height
        detect april tag
        if successfully detected, move to tracking
        """

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600) # Need serial port + baudrate
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

# Switch to autonomous mode
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                             0,
                             220, 0, 0, 0, 0, 0, 0) # idk the rest of the parameters is this right ???

# Prearm check - returns "MAV_RESULT_ACCEPTED" if successful, "MAV_RESULT_TEMPORARILY_REJECTED" if system is already armed
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                             0)

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

# Request camera capture status
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                             0,
                             527, 1, 0, 0, 0, 0, 0) # MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS


# Start running sensors
while # add some condition
    # GPIO Modus (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    # assign GPIO Pins
    GPIO_SIG = 17


    def distance():
        GPIO.setup(GPIO_SIG, GPIO.OUT)
        GPIO.output(GPIO_SIG, 0)

        time.sleep(0.000002)

        # send trigger signal
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
        distance = (duration * 34000) / 2

        return distance


    if __name__ == '__main__':
        try:
            while True:
                distance = distance()
                print("Measured distance = %.1f cm" % distance)
                time.sleep(1)

        # When canceling with CTRL+C, resetting
        except KeyboardInterrupt:
            print("Measurement stopped by user")
            GPIO.cleanup()
