#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
from libutils import arm_and_takeoff, condition_yaw, just_arm
#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=False)

#just_arm(vehicle); 
arm_and_takeoff(vehicle, 10); #takes off the vehicle to a height of 10 metres.
condition_yaw(vehicle, 340, relative=False, anticlock = False, wait_till_complete=0.5) #turns the vehicle 90 degrees anticlockwise
