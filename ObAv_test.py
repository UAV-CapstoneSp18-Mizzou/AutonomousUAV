import serial
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from numpy import matrixlib # cumsum #Only cumsum if doing a moving average on sensor data

    ## ---------Variables that need to be populated---------
    
avoid = True # Turns on obstacle avoidance when True
detected = True
headlock = True # Continually sets the vehicle heading to be compass north
avoid_dst = 20 # Number of cm away from copter that obstacle avoidance will be triggered

    ## ---------Connecting to Serial---------    

def sersplit(sens): # Function that takes the serial stream and translates into NSEW
    if sens.startswith('#'): #and sens1.endswith('$'): Had to take out endswith due to unknown char
        sens1 = sens.partition('#')[-1].rpartition('$')[0] #removes start and end symbol
        
        n1,s1,e1,w1 = sens1.split(".")
        global n,s,e,w
        print ("n=" + n1) # arduino pin #4
        print ("s=" + s1) # arduino pin #6
        print ("e=" + e1) # arduino pin #7
        print ("w=" + w1) # arduino pin #8
        n = int(n1)
        s = int(s1)
        e = int(e1)
        w = int(w1)

        '''        
        while n < avoid_dst or s < avoid_dst or e < avoid_dst or w < avoid_dst:
            detected = True
            obstacle_sensed(n, s, e, w)
       
        else:
            detected = False
        '''               
    else:
        print ("Improper format or no connection")
    return (n, s, e, w)


    ## ---------Various Functions---------
    
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
        time.sleep(1)
	
def obstacle_sensed(sersplit): # Function to do obstacle avoidance    
            
# trying to lump everything into a single while loop
# there are 15 different combinations, this will address them each individually
# we can try to make it more concise later as we troubleshoot
#    while avoid == True:
#        sersplit(ser.readline())
#        get_bearing(vehicle.location.global_frame, next_waypoint(wp))   # should determine the bearing to the next wp
# If the next wp is north: bearing 0 to 90 or 270 to 360
# If the next wp is south: bearing 90 to 180
    while n < avoid_dst or w < avoid_dst or e < avoid_dst or s < avoid_dst:
        # checking if object detected in 1 DIRECTION
        if n < avoid_dst and w > avoid_dst and e > avoid_dst and s > avoid_dst:
            print ("Object detected north")
            if w > e and w > s:
                send_ned_velocity(0, -0.5, 0, 1)
                print ("Move west")
                sersplit(ser.readline())
            elif e > w and e > s:
                send_ned_velocity(0, 0.5, 0, 1)
                print ("Move east")
                sersplit(ser.readline())
##            elif s > e and s > w:
##                send_ned_velocity(-0.5, 0, 0, 1)
##                print ("Move south")
##                sersplit(ser.readline())
            else:
                send_ned_velocity(0, 0.5, 0, 1)     # sending east in the event that some are equal
                print ("Move east")
                sersplit(ser.readline())
        elif w < avoid_dst and n > avoid_dst and e > avoid_dst and s > avoid_dst:
            print ("Object detected west")
            if n > e and n > s:
                send_ned_velocity(0.5, 0, 0, 1)
                print ("Move north")
                sersplit(ser.readline())
##            elif e > n and e > s:
##                send_ned_velocity(0, 0.5, 0, 1)
##                print ("Move east")
##                sersplit(ser.readline())
            elif s > e and s > n:
                send_ned_velocity(-0.5, 0, 0, 1)
                print ("Move south")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0.5, 0, 0, 1)     # sending north in the event that some are equal
                print ("Move north")
                sersplit(ser.readline())
        elif e < avoid_dst and n > avoid_dst and w > avoid_dst and s > avoid_dst:
            print ("Object detected east")
            if n > w and n > s:
                send_ned_velocity(0.5, 0, 0, 1)
                print ("Move north")
                sersplit(ser.readline())
##            elif w > n and w > s:
##                send_ned_velocity(0, -0.5, 0, 1)
##                print ("Move west")
##                sersplit(ser.readline())
            elif s > n and s > w:
                send_ned_velocity(-0.5, 0, 0, 1)
                print ("Move south")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0.5, 0, 0, 1)     # sending north in the event that some are equal
                print ("Move north")
                sersplit(ser.readline())
        elif s < avoid_dst and n > avoid_dst and e > avoid_dst and w > avoid_dst:
            print ("Object detected south")
            if e > w and e > n:
                send_ned_velocity(0, 0.5, 0, 1)
                print ("Move east")
                sersplit(ser.readline())
##            elif n > e and n > w:
##                send_ned_velocity(0.5, 0, 0, 1)
##                print ("Move north")
##                sersplit(ser.readline())
            elif w > e and w > n:
                send_ned_velocity(0, -0.5, 0, 1)
                print ("Move west")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0, 0.5, 0, 1)     # sending east in the event that some are equal
                print ("Move east")
                sersplit(ser.readline())

        # checking if object detected in 2 DIRECTIONS
        elif n < avoid_dst and e < avoid_dst and w > avoid_dst and s > avoid_dst:
            print ("Object detected north and east")
            if w > s:
                send_ned_velocity(0, -0.5, 0, 1)
                print ("Move west")
                sersplit(ser.readline())
            elif s > w:
                send_ned_velocity(-0.5, 0, 0, 1)
                print ("Move south")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0, -0.5, 0, 1)
                print ("Move west")
                sersplit(ser.readline())
        elif n < avoid_dst and w < avoid_dst and e > avoid_dst and s > avoid_dst:
            print ("Object detected north and west")
            if e > s:
                send_ned_velocity(0, 0.5, 0, 1)
                print ("Move east")
                sersplit(ser.readline())
            elif s > e:
                send_ned_velocity(-0.5, 0, 0, 1)
                print ("Move south")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0, 0.5, 0, 1)
                print ("Move east")
                sersplit(ser.readline())
        elif n < avoid_dst and s < avoid_dst and e > avoid_dst and w > avoid_dst:
            print ("Object detected north and south")
            if e > w:
                send_ned_velocity(0, 0.5, 0, 1)
                print ("Move east")
                sersplit(ser.readline())
            elif w > e:
                send_ned_velocity(0, -0.5, 0, 1)
                print ("Move west")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0, 0.5, 0, 1)
                print ("Move east")
                sersplit(ser.readline())
        elif e < avoid_dst and w < avoid_dst and n > avoid_dst and s > avoid_dst:
            print ("Object detected east and west")
            if n > s:
                send_ned_velocity(0.5, 0, 0, 1)
                print ("Move north")
                sersplit(ser.readline())
            elif s > n:
                send_ned_velocity(-0.5, 0, 0, 1)
                print ("Move south")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0.5, 0, 0, 1)
                print ("Move north")
                sersplit(ser.readline())
        elif e < avoid_dst and s < avoid_dst and n > avoid_dst and w > avoid_dst:
            print ("Object detected east and south")
            if n > w:
                send_ned_velocity(0.5, 0, 0, 1)
                print ("Move north")
                sersplit(ser.readline())
            elif w > n:
                send_ned_velocity(0, -0.5, 0, 1)
                print ("Move west")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0.5, 0, 0, 1)
                print ("Move north")
                sersplit(ser.readline())
        elif w < avoid_dst and s < avoid_dst and n > avoid_dst and e > avoid_dst:
            print ("Object detected west and south")
            if n > e:
                send_ned_velocity(0.5, 0, 0, 1)
                print ("Move north")
                sersplit(ser.readline())
            elif e > n:
                send_ned_velocity(0, 0.5, 0, 1)
                print ("Move east")
                sersplit(ser.readline())
            else:
                send_ned_velocity(0.5, 0, 0, 1)
                print ("Move north")
                sersplit(ser.readline())

        # checking if object detected in 3 DIRECTIONS
        elif n < avoid_dst and e < avoid_dst and w< avoid_dst and s > avoid_dst:
            print ("Object detected north, east, and west")
            send_ned_velocity(-0.5, 0, 0, 1)
            print ("Move south")
            sersplit(ser.readline())
        elif n < avoid_dst and e < avoid_dst and s < avoid_dst and w > avoid_dst:
            print ("Object detected north, east, and south")
            send_ned_velocity(0, -0.5, 0, 1)
            print ("Move west")
            sersplit(ser.readline())
        elif n < avoid_dst and s < avoid_dst and w < avoid_dst and e > avoid_dst:
            print ("Object detected north, south, and west")
            send_ned_velocity(0, 0.5, 0, 1)
            print ("Move east")
            sersplit(ser.readline())
        elif e < avoid_dst and s < avoid_dst and w < avoid_dst and n > avoid_dst:
            print ("Object detected east, south, and west")
            send_ned_velocity(0.5, 0, 0, 1)
            print ("Move north")
            sersplit(ser.readline())

        # checking if object detected in 4 DIRECTIONS
        elif n < avoid_dst and e < avoid_dst and s < avoid_dst and w < avoid_dst:
            print ("Object detected north, east, south, and west")
            print ("You are surrounded")
            send_ned_velocity(0, 0, -0.5, 1)
            print ("Move up")       # moves up if it is surrounded to go over the objects
            sersplit(ser.readline())

        else:
            print ("The situation did not fall into any category--TROUBLESHOOT OR CHECK IF DIR == avoid_dst")
            sersplit(ser.readline())

    ## ---------Connecting to the Pixhawk---------

vehicle=connect('/dev/serial0', baud=57600, wait_ready=True) # Connects to Pixhawk
#while True:
print " Autopilot Firmware version: %s" % vehicle.version
vehicle.mode = VehicleMode("GUIDED")
print vehicle.is_armable

    ## ---------Connecting to Serial---------

ser=serial.Serial("/dev/ttyUSB0", 9600, timeout=5)  # Opens serial stream
while avoid == True:
    sersplit(ser.readline()) # Note: currently must restart shell to end stream. Don't close port
    obstacle_sensed(sersplit)
