import pygame
import socket
import json
import time

# --- NETWORK SETTINGS ---
# We now talk to the Broker instead of the Serial Port
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# --- MOVEMENT SETTINGS ---
STEADY_SPEED = 140   
FAST_SPEED = 200     
TURN_SENSITIVITY = 85 

# Setup UDP Socket for sending commands to the Broker
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick found! Ensure the USB dongle is connected.")
    exit()

j = pygame.joystick.Joystick(0)
j.init()

def get_stepped_velocity(axis_val):
    abs_val = abs(axis_val)
    if abs_val < 0.1:
        return 0
    sign = 1 if axis_val > 0 else -1
    if abs_val < 0.9:
        return sign * STEADY_SPEED
    else:
        return sign * FAST_SPEED

try:
    print(f"--- Rover1 Teleop (via Broker) Active ---")
    while True:
        pygame.event.pump()
        
        # 1. Linear (Left Stick Y)
        ls_y = -j.get_axis(1) 
        target_v = get_stepped_velocity(ls_y)
        
        # 2. Steering (Right Stick X)
        rs_x = j.get_axis(2) 
        target_w = int(rs_x * TURN_SENSITIVITY) if abs(rs_x) > 0.1 else 0
        
        # 3. DIFFERENTIAL DRIVE KINEMATICS
        l_pwm = max(min(target_v + target_w, 255), -255)
        r_pwm = max(min(target_v - target_w, 255), -255)
        
        # 4. FORMAT FOR BROKER
        # The Broker expects a JSON object with a "cmd" key
        command_str = f"S{l_pwm},{r_pwm}"
        payload = json.dumps({"cmd": command_str})
        
        # 5. SEND VIA UDP
        sock.sendto(payload.encode(), (UDP_IP, UDP_PORT))
        
        print(f"SENDING TO BROKER: {command_str}      ", end='\r')
        time.sleep(0.05)

except KeyboardInterrupt:
    # Emergency Stop via Broker
    stop_payload = json.dumps({"cmd": "S0,0"})
    sock.sendto(stop_payload.encode(), (UDP_IP, UDP_PORT))
    print("\nTeleop Halted. Emergency Brake Sent to Broker.")