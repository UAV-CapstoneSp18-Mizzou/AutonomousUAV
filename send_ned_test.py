import serial
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from numpy import matrixlib # cumsum #Only cumsum if doing a moving average on sensor data

    ## ---------Variables that need to be populated---------
    
avoid = True # Turns on obstacle avoidance when True
detected = True
headlock = True # Continually sets the vehicle heading to be compass north


    ## ---------Vehicle Movement Functions---------
    
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(2)


def condition_yaw(heading, relative=False):    # Function to set the heading of the vehicle.
	msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,heading,0,1,0,0,0,0)
	vehicle.send_mavlink(msg)

    ## ---------Connecting to the Pixhawk---------

vehicle=connect('/dev/serial0', baud=57600, wait_ready=True) # Connects to Pixhawk
#while True:
print " Autopilot Firmware version: %s" % vehicle.version
vehicle.mode = VehicleMode("GUIDED")
print vehicle.is_armable
# while headlock == True:
condition_yaw(0, True)
print "Heading locked."

    ## ---------Connecting to Serial---------

ser=serial.Serial("/dev/ttyUSB0", 9600, timeout=5)  # Opens serial stream


    ## ---------send_ned_velocity test---------
def send_ned_test():
    vehicle.mode = VehicleMode("GUIDED")
    condition_yaw(0, True)
    send_ned_velocity(1,0,0,2)
    vehicle.mode = VehicleMode("POSHOLD")
    time.sleep(10)

for i in [0, 1, 2, 3]:
    send_ned_test()
    print "Moving"
