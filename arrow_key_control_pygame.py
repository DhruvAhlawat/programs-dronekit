import pygame
from pygame.locals import *
import libutils as lb



# Initialize Pygame
pygame.init()
# Set up the display
window = pygame.display.set_mode((400, 400))

# Initialize key states
key_states = {
    K_UP: False,
    K_DOWN: False,
    K_LEFT: False,
    K_RIGHT: False,
}

running = True
vehicle = lb.connectMyCopter(); 
lb.arm_and_takeoff(vehicle,7); 
ground_speed = 0.5; 

while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        if event.type == KEYDOWN:
            if event.key in key_states:
                key_states[event.key] = True
        if event.type == KEYUP:
            if event.key in key_states:
                key_states[event.key] = False
                print("key released: ", event.key)

    # Check if arrow keys are held down or released
    if key_states[K_UP]:
        lb.set_velocity_body(vehicle, ground_speed, 0, 0);
        print("Up key held down")
    elif key_states[K_DOWN]:
        lb.set_velocity_body(vehicle, -ground_speed, 0, 0);
        print("Down key held down")
    elif key_states[K_LEFT]:
        lb.set_velocity_body(vehicle, 0, -ground_speed, 0);
        print("Left key held down")
    elif key_states[K_RIGHT]:
        lb.set_velocity_body(vehicle, 0, ground_speed, 0);
        print("Right key held down")
    else:
        lb.set_velocity_body(vehicle, 0, 0, 0);
        print("No key held down")
    pygame.display.update()

pygame.quit()
