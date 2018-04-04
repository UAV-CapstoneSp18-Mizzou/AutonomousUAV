## Spring 2018 Capstone - Autonomous Multirotor UAV
## Blake Bremer, James Murphy, Nick Bush, Matt Drury, Brad Dahlman

##      --------Revision Log-----------
# 3/26/18 - Version 1.0 created
# 3/27/18 - sersplit() function created and tested with Arduino Nano
# 3/28/18 3:00 - added loc_callback(), heading_callback(), and Pixhawk connections
# 3/28/18 13:07 - added arm_and_takeoff() and populated variables
# 3/30/18 - edited next_waypoint() so that it might actually work
#
#
#

import serial
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from numpy import matrixlib, cumsum #Only cumsum if doing a moving average on sensor data

    ## ---------Variables that need to be populated---------
    
avoid = True # Turns on obstacle avoidance when True
headlock = True # Continually sets the vehicle heading to be compass north
avoid_dst = 100 # Number of cm away from copter that obstacle avoidance will be triggered

targalt = 6 # target altitude that vehicle will fly at (meters)

# lat1 = 38.9260971 # Hardcoded waypoint targets
# lon1 = -92.3299813 # Need to figure out how to access these without hardcode
# lat2 = 38.9262390 # Something like a GUI
# lon2 = -92.3293698 # Or possibly a simple command via wifi connection
# lat3 = 38.9266980 # Wifi might be easiest since this copter (queen) will host a network
# lon3 = -92.3296380 # GUI with wireless mouse / keyboard might be quickest
# lat4 = 38.9265395 # Though it would be rather archaic
# lon4 = -92.3302549 #  <---- currently set to a rough square on Hinkson

wplist = [[1,38.9260971,-92.3299813,targalt], # list format: [wp#,lat,lon,alt]
        [2,38.9262390,-92.3293698,targalt],
        [3,38.9266980,-92.3296380,targalt],
        [4,38.9265395,-92.3302549,targalt]]

    ## ---------Various Functions---------

def sersplit(sens): # Function that takes the serial stream and translates into NSEW
    if sens.startswith('#'): #and sens1.endswith('$'): Had to take out endswith due to unknown char
        sens1 = sens.partition('#')[-1].rpartition('$')[0] #removes start and end symbol
        global n,s,e,w
        n,s,e,w = sens1.split(".")
        print ("n=" + n) # arduino pin #5
        print ("s=" + s) # arduino pin #6
        print ("e=" + e) # arduino pin #7
        print ("w=" + w) # arduino pin #8
        
        while n < avoid_dst or s < avoid_dst or e < avoid_dst or w < avoid_dst:
            detected = True
            obstacle_sensed(n, s, e, w)
        
        else:
            detected = False
            
    else:
        print ("Improper format or no connection")
        
    if detected = True:
        return True
    elif detected = False
        return False

def location_callback(self, attr_name, value): # gets location and returns string
    #global locstr
    loc = str(value)
    loclist = loc.split('=|,') #parses the value returned by function
    lat=loclist[1] #pulls latitude item from list
    lon=loclist[3] #pulls longitude item from list
    alt=loclist[5] #pulls altitude item from list
    print "Location: " + loc # print for testing
    

def heading_callback(self, attr_name, value): #gets heading and returns string
    global headstr
    headstr= str(value) # places heading in headstr var
    print "Heading: " + headstr # for testing


    ## ---------Vehicle Movement Functions---------
	
def condition_yaw(heading, relative=False):    # Function to set the heading of the vehicle.
	''' May have something like while true: condition_yaw(0, False)
		This would keep vehicle locked on true north for entire flight
	'''
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def arm_and_takeoff(targalt):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(targalt) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=targalt*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def next_waypoint(wp): # Function to go to next wp until within 2m
    wpnum = 0
    wp = wplist[wpnum[1:3]]
    while detected is False: # detected should return false while no readings
       
        while get_distance_metres(location_callback(), wplist[wpnum[1:3]]) >= 2: # want to measure diff between current loc and next wp
            Vehicle.simple_goto(wplist[wpnum[1:3]])
            
            if wplist[wpnum[0]]-len(wplist) > 0: # Determines if this is the last waypoint
                print 'Traveling to waypoint: ' + str(wplist[wpnum])
            
            elif wplist[wpnum[0]]-len(wplist) = 0: # Determines if this is the last waypoint
                print 'Traveling to final waypoint: ' + str(wplist[wpnum])
        
        else:
            print 'Arrived at wapoint: ' + str(wplist[wpnum])
            if wplist[wpnum[0]]-len(wplist) > 0:
                wpnum = wpnum + 1
                print 'Proceding to waypoint: ' + str(wplist[wpnum])
            else:
                print 'Final wapypoint reached. Switching to Position Hold.'
                vehicle.mode = VehicleMode("PosHold") # Might not be correct mode name. Double check
        
    else:
        print 'Obstacle detected. Delaying travel to wapoint #' + str(wpnum)

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;        
	
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
	
def obstacle_sensed(n, s, e, w): # Function to do obstacle avoidance
    #while n < avoid_dst:        
    #while s < avoid_dst:        
    #while e < avoid_dst:  
    #while w < avoid_dst:
    '''
    
    # assuming UAV is continuously set to true north
    '''
'''
# CHECK FORWARD/NORTH
	while n < avoid_dst:
		if w > avoid_dst or e > avoid_dst:
			if w > e:
				print ("move west")
				send_ned_velocity(0, -0.5, 0, 1)
			else:
				print ("move east")
				send_ned_velocity(0, 0.5, 0, 1)
	    	else:
			print ("move south")
			send_ned_velocity(-0.5, 0, 0, 1)
			
# CHECK LEFT/WEST			
	while w < avoid_dst:
		if n > avoid_dst or e > avoid_dst:
			if n > e:
				print ("move north")
				send_ned_velocity(0.5, 0, 0, 1)
			else:
				print ("move north")
				send_ned_velocity(0.5,0, 0, 1)
		else:
			print ("move east")
			send_ned_velocity(0, 0.5, 0, 1)
'''
			
# trying to lump everything into a single while loop
# there are 15 different combinations, this will address them each individually
# we can try to make it more concise later as we troubleshoot
	while avoid == True:
		get_bearing(vehicle.location.global_frame, next_waypoint(wp))	# should determine the bearing to the next wp
		while n < avoid_dst or w < avoid_dst or e < avoid_dst or s < avoid_dst:
# If the next wp is north: bearing 0 to 90 or 270 to 360
# If the next wp is south: bearing 90 to 180
			# checking if object detected in 1 DIRECTION
			if n < avoid_dst and w > avoid_dst and e > avoid_dst and s > avoid_dst:
				print ("Object detected north")
				if w > e and w > s:
					send_ned_velocity(0, -0.5, 0, 1)
					print ("Move west")
				elif e > w and e > s:
					send_ned_velocity(0, 0.5, 0, 1)
					print ("Move east")
				elif s > e and s > w:
					send_ned_velocity(-0.5, 0, 0, 1)
					print ("Move south")
				else:
					send_ned_velocity(0, 0.5, 0, 1)		# sending east in the event that some are equal
			elif w < avoid_dst and n > avoid_dst and e > avoid_dst and s > avoid_dst:
				print ("Object detected west")
				if n > e and n > s:
					send_ned_velocity(0.5, 0, 0, 1)
					print ("Move north")
				elif e > n and e > s:
					send_ned_velocity(0, 0.5, 0, 1)
					print ("Move east")
				elif s > e and s > n:
					send_ned_velocity(-0.5, 0, 0, 1)
					print ("Move south")
				else:
					send_ned_velocity(0.5, 0, 0, 1)		# sending north in the event that some are equal
			elif e < avoid_dst and n > avoid_dst and w > avoid_dst and s > avoid_dst:
				print ("Object detected east")
				if n > w and n > s:
					send_ned_velocity(0.5, 0, 0, 1)
					print ("Move north")
				elif w > n and w > s:
					send_ned_velocity(0, -0.5, 0, 1)
					print ("Move west")
				elif s > n and s > w:
					send_ned_velocity(-0.5, 0, 0, 1)
					print ("Move south")
				else:
					send_ned_velocity(0.5, 0, 0, 1)		# sending north in the event that some are equal
			elif s < avoid_dst and n > avoid_dst and e > avoid_dst and w > avoid_dst:
				print ("Object detected south")
				if n > e and n > w:
					send_ned_velocity(0.5, 0, 0, 1)
					print ("Move north")
				elif e > w and e > n:
					send_ned_velocity(0, 0.5, 0, 1)
					print ("Move east")
				elif w > e and w > n:
					send_ned_velocity(0, -0.5, 0, 1)
					print ("Move west")
				else:
					send_ned_velocity(0, 0.5, 0, 1)		# sending east in the event that some are equal

			# checking if object detected in 2 DIRECTIONS
			elif n < avoid_dst and e < avoid_dst and w > avoid_dst and s > avoid_dst:
				print ("Object detected north and east")
				if w > s:
					send_ned_velocity(0, -0.5, 0, 1)
					print ("Move west")
				elif s > w:
					send_ned_velocity(-0.5, 0, 0, 1)
					print ("Move south")
				else:
					send_ned_velocity(0, -0.5, 0, 1)
			elif n < avoid_dst and w < avoid_dst and e > avoid_dst and s > avoid_dst:
				print ("Object detected north and west")
				if e > s:
					send_ned_velocity(0, 0.5, 0, 1)
					print ("Move east")
				elif s > e:
					send_ned_velocity(-0.5, 0, 0, 1)
					print ("Move south")
				else:
					send_ned_velocity(0, 0.5, 0, 1)
			elif n < avoid_dst and s < avoid_dst and e > avoid_dst and w > avoid_dst:
				print ("Object detected north and south")
				if e > w:
					send_ned_velocity(0, 0.5, 0, 1)
					print ("Move east")
				elif w > e:
					send_ned_velocity(0, -0.5, 0, 1)
					print ("Move west")
				else:
					send_ned_velocity(0, 0.5, 0, 1)
			elif e < avoid_dst and w < avoid_dst and n > avoid_dst and s > avoid_dst:
				print ("Object detected east and west")
				if n > s:
					send_ned_velocity(0.5, 0, 0, 1)
					print ("Move north")
				elif s > n:
					send_ned_velocity(-0.5, 0, 0, 1)
					print ("Move south")
				else:
					send_ned_velocity(0.5, 0, 0, 1)
			elif e < avoid_dst and s < avoid_dst and n > avoid_dst and w > avoid_dst:
				print ("Object detected east and south")
				if n > w:
					send_ned_velocity(0.5, 0, 0, 1)
					print ("Move north")
				elif w > n:
					send_ned_velocity(0, -0.5, 0, 1)
					print ("Move west")
				else:
					send_ned_velocity(0.5, 0, 0, 1)
			elif w < avoid_dst and s < avoid_dst and n > avoid_dst and e > avoid_dst:
				print ("Object detected west and south")
				if n > e:
					send_ned_velocity(0.5, 0, 0, 1)
					print ("Move north")
				elif e > n:
					send_ned_velocity(0, 0.5, 0, 1)
					print ("Move east")
				else:
					send_ned_velocity(0.5, 0, 0, 1)

			# checking if object detected in 3 DIRECTIONS
			elif n < avoid_dst and e < avoid_dst and w< avoid_dst and s > avoid_dst:
				print ("Object detected north, east, and west")
				send_ned_velocity(-0.5, 0, 0, 1)
				print ("Move south")
			elif n < avoid_dst and e < avoid_dst and s < avoid_dst and w > avoid_dst:
				print ("Object detected north, east, and south")
				send_ned_velocity(0, -0.5, 0, 1)
				print ("Move west")
			elif n < avoid_dst and s < avoid_dst and w < avoid_dst and e > avoid_dst:
				print ("Object detected north, south, and west")
				send_ned_velocity(0, 0.5, 0, 1)
				print ("Move east")
			elif e < avoid_dst and s < avoid_dst and w < avoid_dst and n > avoid_dst:
				print ("Object detected east, south, and west")
				send_ned_velocity(0.5, 0, 0, 1)
				print ("Move north")

			# checking if object detected in 4 DIRECTIONS
			elif n < avoid_dst and e < avoid_dst and s < avoid_dst and w < avoid_dst:
				print ("Object detected north, east, south, and west")
				print ("You are surrounded")
				send_ned_velocity(0, 0, -0.5, 1)
				print ("Move up")		# moves up if it is surrounded to go over the objects

			else:
				print ("The situation did not fall into any category--TROUBLESHOOT")
			
'''
# CHECK FORWARD/NORTH
	while n < avoid_dst:
		# object north
		if w > avoid_dst and e > avoid_dst and s > avoid_dst:
			if w > e and w > s
				print ("move west")
				send_ned_velocity(0, -0.5, 0, 1)
			elif e > w and e > s
				print ("move east")
				send_ned_velocity(0, 0.5, 0, 1)
			elif s > w and s > e
				print ("move south")
				send_ned_velocity(-0.5, 0, 0, 1)
			else:
				# moves right if some of the statements are equal
				print ("move east")
				send_ned_velocity(0, 0.5, 0,1)
		# object north and south
		elif w > avoid_dst and e > avoid_dst and s < avoid_dst:
			if w > e:
				print ("move west")
				send_ned_velocity(0, -0.5, 0, 1)
			else:
				print ("move east")
				send_ned_velocity(0, 0.5, 0, 1)
		# object north and east
		elif w > avoid_dst and e < avoid_dst and s > avoid_dst:
			if s > w:
				print ("move south")
				send_ned_velocity(-0.5, 0, 0, 1)
			else:
				print ("move west")
				send_ned_velocity(0, -0.5, 0, 1)
		# object north and west
		elif w < avoid_dst and e > avoid_dst and s > avoid_dst:
			if s > e:
				print ("move south")
				send_ned_velocity(-0.5, 0, 0, 1)
			else:
				print ("move east")
				send_ned_velocity(0, 0.5, 0, 1)
		# object north, west, east
		elif w < avoid_dst and e < avoid_dst and s > avoid_dst:
			print ("move south")
			send_ned_velocity(-0.5, 0, 0, 1)
		# object north, south, west
		elif w < avoid_dst and s < avoid_dst and e > avoid_dst:
			print ("move east")
			send_ned_velocity(0, 0.5, 0, 1)
		# object north, south, east
		elif s < avoid_dst and e < avoid_dst and w > avoid_dst:
			print ("move west")
			send_ned_velocity(0, -0.5, 0, 1)
		# every side detects obstacles within safe distance
		elif w < avoid_dst and e < avoid_dst and s < avoid_dst:
			print ("surrounded")
'''	
		


        
# CODE TO FLY    

ser=serial.Serial("/dev/ttyUSB0", 9600, timeout=5)  # Opens serial stream
while avoid = True:
    sersplit(ser.readline()) # Note: currently must restart shell to end stream. Don't close port
else detected = False

while headlock = True:
	condition_yaw(0, True)

    ## ---------Connecting to the Pixhawk---------

vehicle=connect('/dev/serial0', baud=57600, wait_ready=True) # Connects to Pixhawk

    ## ---------Adding location and heading listeners---------

vehicle.add_attribute_listener('location.global_frame', location_callback) # listener constantly calls location
vehicle.add_attribute_listener('heading', heading_callback) # listener constantly calls heading
