import libutils as lb

#from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
from time import sleep
import math
#Set up option parsing to get connection string
import argparse  

vehicle = lb.connectMyCopter();

lb.just_arm(vehicle); #this should arm the vehicle.

#then we wait for a few seconds

sleep(10);

lb.disarm(vehicle); 
