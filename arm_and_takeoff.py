import libutils as lb
import sys
from time import sleep
import math
#Set up option parsing to get connection string
import argparse  

vehicle = lb.connectMyCopter();

#lb.just_arm(vehicle); #this should arm the vehicle.

#then we wait for a few seconds
lb.arm_and_takeoff(vehicle,7); 
while(True):
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
lb.disarm(vehicle); 
