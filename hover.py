from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600) # Need serial port + baudrate
master.reboot_autopilot() # reboots ArduSub board

master = mavutil.mavlink_connection('udpin:0.0.0.0:9000') # which is right???? internet not help pls idk which master we need
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

        """
        Do we need this? idk help
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
                             220, 0, 0, 0, 0, 0, 0) # idk the rest of the parameters is this right ??? double check it emily
                             # 220 = system is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)

# Arming vehicle
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             0,
                             1, 0, 0, 0, 0, 0, 0)

# Rise to specified height
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL,
                             0,
                             0, 0, 0, 0, 0, 0, 0.5) # help is 0.5 m OK?? do we even operate in meters????

# idk some specified amount of time like a min?
time.sleep(60)

# Landing
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL,
                             0,
                             0, 0, descentrate, 0, 0, 0, 0) # lol what is our descent rate

# Disarm
master.mav.command_long_send(master.target_system,
                             master.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             0,
                             0, 0, 0, 0, 0, 0, 0)



