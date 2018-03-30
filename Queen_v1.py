## Spring 2018 Capstone - Autonomous Multirotor UAV
## Blake Bremer, James Murphy, Nick Bush, Matt Drury, Brad Dahlman

##      --------Revision Log-----------
# 3/26/18 - Version 1.0 created
# 3/27/18 - sersplit() function created and tested with Arduino Nano
# 3/28/18 3:00 - added loc_callback(), heading_callback(), and Pixhawk connections
# 3/28/18 13:07 - added arm_and_takeoff() and populated variables
#
#
#
#

import serial
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from numpy import matrixlib

    ## ---------Variables that need to be populated---------

targalt = 6 # target altitude that vehicle will fly at (meters)

lat1 = 38.9260971 # Hardcoded waypoint targets
lon1 = -92.3299813 # Need to figure out how to access these without hardcode
lat2 = 38.9262390 # Something like a GUI
lon2 = -92.3293698 # Or possibly a simple command via wifi connection
lat3 = 38.9266980 # Wifi might be easiest since this copter (queen) will host a network
lon3 = -92.3296380 # GUI with wireless mouse / keyboard might be quickest
lat4 = 38.9265395 # Though it would be rather archaic
lon4 = -92.3302549 #  <---- currently set to a rough square on Hinkson

wplist = [[1,38.9260971,-92.3299813,targalt], # list format: [wp#,lat,lon,alt]
        [2,38.9262390,-92.3293698,targalt],
        [3,38.9266980,-92.3296380,targalt],
        [4,38.9265395,-92.3302549,targalt]]

    ## ---------Various Functions---------

def sersplit(sens): # Function that takes the serial stream and translates into nsew
    if sens.startswith('#'): #and sens1.endswith('$'): Had to take out endswith due to unknown char
        sens1 = sens.partition('#')[-1].rpartition('$')[0] #removes start and end symbol
        global n,s,e,w
        n,s,e,w = sens1.split(".")
        print ("n=" + n) # arduino pin #5
        print ("s=" + s) # arduino pin #6
        print ("e=" + e) # arduino pin #7
        print ("w=" + w) # arduino pin #8
    else:
        print ("Improper format or no connection")

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

def next_waypoint(wp) # Function to go to next wp until within 2m
    wpnum = 0
    wp = wplist[wpnum[1:3]]
    while obstacle_sensed() is False: #obstacle_sensed() should return false while no readings
       
        while get_distance_metres(location_callback(), wplist[wpnum[1:3]]) >= 2: # want to measure diff between current loc and next wp
            Vehicle.simple_goto(wplist[wpnum[1:3]])
            
            if wplist[wpnum[0]]-len(wplist) > 0:
                print 'Traveling to waypoint: ' + str(wplist[wpnum])
            
            elif wplist[wpnum[0]]-len(wplist) = 0:
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

ser=serial.Serial("/dev/ttyUSB0", 9600, timeout=5)  # Opens serial stream
while True:
    sersplit(ser.readline()) # Note: currently must restart shell to end stream. Don't close port

    ## ---------Connecting to the Pixhawk---------

vehicle=connect('/dev/serial0', baud=57600, wait_ready=False) # Connects to Pixhawk

    ## ---------Adding location and heading listeners---------

vehicle.add_attribute_listener('location.global_frame', location_callback) # listener constantly calls location
vehicle.add_attribute_listener('heading', heading_callback) # listener constantly calls heading
