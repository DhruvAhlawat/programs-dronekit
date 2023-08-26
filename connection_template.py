from dronekit import connect, VehicleMode, LocationGlobalRelative,APIException
import time
import socket
#import exceptions
import math
import argparse


def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands');
    parser.add_argument('--connect');
    args = parser.parse_args();
    connection_string = args.connect
    print("starting to connect to ", connection_string);  
    vehicle = connect(connection_string, wait_ready=False);
    print("conencted to vehicle"); 
    return vehicle; 

###>> python3 connect_template.py --connect 127.0.0.1:14550


vehicle = connectMyCopter(); 
print(vars(vehicle)); 

	

	




