import serial
import threading
import math
import time

class RoverController:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.CPR = 1980.0            # Counts Per Revolution
        self.WHEEL_DIAMETER = 0.065  # 65mm in meters
        self.WHEEL_CIRC = self.WHEEL_DIAMETER * math.pi
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.running = True
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            # Start a background thread to listen for telemetry (ticks)
            self.receiver_thread = threading.Thread(target=self._listen, daemon=True)
            self.receiver_thread.start()
            print(f"HAL: Connected to ESP32 on {port}")
        except Exception as e:
            self.ser = None
            print(f"HAL: Virtual Mode Active (Serial Error: {e})")

    def _listen(self):
        """Background thread to parse incoming telemetry from ESP32."""
        while self.running and self.ser:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("STATS:"):
                    # Expecting "STATS:L_TICKS:R_TICKS"
                    parts = line.split(':')
                    if len(parts) == 3:
                        self.left_ticks = int(parts[1])
                        self.right_ticks = int(parts[2])

    def move(self, velocity, omega):
        """Sends the standard Intent protocol command."""
        command = f"MOVE:{velocity}:{omega}\n"
        if self.ser:
            self.ser.write(command.encode())
        else:
            pass # Silence virtual mode output to keep terminal clean

    def get_distance_meters(self):
        """Standard math for your 65mm wheels."""
        avg_ticks = (self.left_ticks + self.right_ticks) / 2.0
        return (avg_ticks / self.CPR) * self.WHEEL_CIRC

    def stop(self):
        self.move(0, 0)
        self.running = False
        if self.ser:
            self.ser.close()