import pygame
import serial
import time

# --- HARDWARE SETTINGS ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# --- MOVEMENT SETTINGS ---
STEADY_SPEED = 140   
FAST_SPEED = 200     
TURN_SENSITIVITY = 85 

try:
    # Increased timeout slightly for better stability
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to ESP32 on {SERIAL_PORT}")
except Exception as e:
    ser = None
    print(f"ESP32 not found ({e}). Running in VIRTUAL mode.")

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
    print(f"--- Rover1 Teleop Active ---")
    while True:
        pygame.event.pump()
        
        # 1. Linear (Left Stick Y - Axis 1)
        ls_y = -j.get_axis(1) 
        target_v = get_stepped_velocity(ls_y)
        
        # 2. Steering (Right Stick X - Axis 2)
        rs_x = j.get_axis(2) 
        if abs(rs_x) < 0.1:
            target_w = 0
        else:
            # target_w is the speed difference between wheels
            target_w = int(rs_x * TURN_SENSITIVITY)
        
        # 3. DIFFERENTIAL DRIVE KINEMATICS
        # Mixing V and W to get specific motor PWM values
        l_pwm = target_v + target_w
        r_pwm = target_v - target_w
        
        # 4. CONSTRAIN (Ensure we stay within -255 to 255)
        l_pwm = max(min(l_pwm, 255), -255)
        r_pwm = max(min(r_pwm, 255), -255)
        
        # 5. NEW COMMAND FORMAT: S[Left],[Right]
        command = f"S{l_pwm},{r_pwm}\n"
        
        # Print intent for debugging
        print(f"V:{target_v} W:{target_w} -> SENDING: {command.strip()}      ", end='\r')
        
        if ser:
            ser.write(command.encode())
            ser.flush() 
            
        time.sleep(0.05)

except KeyboardInterrupt:
    if ser:
        ser.write(b"MOVE:0:0\n")
    print("\nTeleop Halted. Emergency Brake Applied.")
