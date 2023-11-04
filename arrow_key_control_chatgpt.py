import keyboard
from dronekit import connect, VehicleMode

# Connect to your drone's IP and port (e.g., udp:127.0.0.1:14550 for a simulator)
connection_string = "udp:127.0.0.1:14550"  # Update with your drone's connection details
vehicle = connect(connection_string, wait_ready=True)

# Arm the drone
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.is_armable:
    print("Waiting for the vehicle to become armable...")
    keyboard.wait("esc")

vehicle.armed = True
while not vehicle.armed:
    print("Arming the vehicle...")
    keyboard.wait("esc")

print("Vehicle armed and ready to fly!")

# Takeoff to a desired altitude
target_altitude = 10  # Replace with your desired altitude in meters
vehicle.simple_takeoff(target_altitude)

while True:
    print("Altitude: {} meters".format(vehicle.location.global_relative_frame.alt))
    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("Altitude reached")
        break

# Set initial values for throttle, yaw, pitch, and roll
throttle = 0.0
yaw = 0.0
pitch = 0.0
roll = 0.0

# Constants for control sensitivity
SPEED = 0.1  # Adjust this to change the control sensitivity

try:
    while True:
        key_event = keyboard.read_event(suppress=False)

        if key_event.name == "up":
            throttle = min(throttle + SPEED, 1.0)
        elif key_event.name == "down":
            throttle = max(throttle - SPEED, 0.0)
        elif key_event.name == "left":
            yaw = -SPEED
        elif key_event.name == "right":
            yaw = SPEED

        # Send control commands to the drone
        vehicle.channels.overrides['3'] = int((throttle + 1) * 1000)  # Throttle (channel 3)
        vehicle.channels.overrides['4'] = int((yaw + 1) * 1000)  # Yaw (channel 4)
        vehicle.channels.overrides['2'] = int((pitch + 1) * 1000)  # Pitch (channel 2)
        vehicle.channels.overrides['1'] = int((roll + 1) * 1000)  # Roll (channel 1)

except KeyboardInterrupt:
    pass

# Disarm and close the connection
vehicle.armed = False
vehicle.close()
