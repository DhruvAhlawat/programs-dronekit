#!/usr/bin/env python
"""
This example shows how to move/direct Copter and send commands
in GUIDED_NOGPS mode using DroneKit Python.
Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
Always watch the drone movement, and make sure that you are in dangerless environment.
Land the drone as soon as possible when it shows any unexpected behavior.
"""
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import libutils as lb
# Set up option parsing to get connection string
import argparse

# Connect to the Vehicle
vehicle =  lb.connectMyCopter(); # Connects my copter to the vehicle.

# Take off 2.5m in GUIDED_NOGPS mode.
lb.arm_and_takeoff_nogps(vehicle, 2.5)

# Hold the position for 3 seconds.
print("Hold position for 3 seconds")
lb.set_attitude(vehicle,duration = 3)

# Uncomment the lines below for testing roll angle and yaw rate.
# Make sure that there is enough space for testing this.

# set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
print("setting yaw")
lb.set_attitude(vehicle, yaw_rate = 90, thrust = 0.5, duration = 10,use_yaw_rate=True); 
print("stopping and waiting for 2 second");
time.sleep(2); 
lb.set_attitude(vehicle, yaw_rate = -90, thrust = 0.5, duration = 10,use_yaw_rate=True);

# Move the drone forward and backward.
# Note that it will be in front of original position due to inertia.
# print("Move forward")
# lb.set_attitude(vehicle,pitch_angle = -5, thrust = 0.5, duration = 3.21)

# print("Move backward")
# lb.set_attitude(vehicle, pitch_angle = 5, thrust = 0.5, duration = 3)


print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
print("Completed")
