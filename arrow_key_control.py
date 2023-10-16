import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import tkinter as tk
import sys;
import libutils as lb

print("Connecting...",end = "");
vehicle = connect(sys.argv[1]);
gnd_speed = 5; 

lb.arm_and_takeoff(vehicle, 10); #Takeoffs to an altitude of 10 metres. 

root = tk.Tk(); 
print("Control the drone with the arrow keys, and press r to return to launch");


def key(event):
    print("pressed key: ", event.keysym); 
    if(event.char == event.keysym): #-- standard keys
        if event.keysm == 'r':
            print("r pressed. RETURN TO LAUNCH initiated")
            vehicle.mode = VehicleMode("RTL"); 
    else:
        if event.keysym == 'Up':
            lb.set_velocity_body(vehicle, gnd_speed, 0, 0);
        elif event.keysym == 'Down':
            lb.set_velocity_body(vehicle, -gnd_speed, 0, 0);
        elif event.keysym == 'Left':
            lb.set_velocity_body(vehicle, 0, -gnd_speed, 0);
        elif event.keysym == 'Right':
            lb.set_velocity_body(vehicle, 0, gnd_speed, 0);


root.bind_all('<Key>', key);
root.mainloop(); 

