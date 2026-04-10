# main.py (Updated for Unified Telemetry)
import serial, socket, json, time, signal, sys, threading

SERIAL_PORT = '/dev/ttyUSB0' # Ensure this matches your ESP32 connection
BAUD_RATE = 115200
UDP_CMD_PORT = 5005
UDP_TELE_PORT = 5006
BROKER_IP = '127.0.0.1'

class RoverBroker:
    def __init__(self):
        self.running = True
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            print(f"BROKER: Connected to LLC on {SERIAL_PORT}")
        except Exception as e:
            print(f"BROKER FATAL ERROR: {e}")
            sys.exit(1)

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind((BROKER_IP, UDP_CMD_PORT))
        self.tele_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def listen_udp(self):
        while self.running:
            try:
                data, addr = self.cmd_sock.recvfrom(1024)
                payload = json.loads(data.decode())
                command = payload.get("cmd", "")
                if command and self.ser.is_open:
                    self.ser.write(f"{command}\n".encode())
            except: continue

    def listen_serial(self):
        """Reads 'T' packets from ESP32 and broadcasts JSON telemetry."""
        while self.running:
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    # DEBUG LINE: Uncomment to see everything coming from the ESP32
                    print(f"RAW: {line}")
                    
                    # PARSER: Updated to look for 'T' (Telemetry) instead of 'E'
                    if line.startswith('T'):
                        # Format: T[Encoders]|[IMU]|[TOFs]
                        sections = line[1:].split('|')
                        if len(sections) == 3:
                            ticks = [int(x) for x in sections[0].split(',')]
                            imu = [float(x) for x in sections[1].split(',')]
                            tofs = [int(x) for x in sections[2].split(',')]
                            
                            telemetry = {
                                "ticks": ticks,
                                "imu": imu,
                                "tofs": tofs,
                                "ts": time.time()
                            }
                            
                            # Broadcast to the Stream/HUD script
                            msg = json.dumps(telemetry).encode()
                            self.tele_sock.sendto(msg, (BROKER_IP, UDP_TELE_PORT))
                except Exception as e:
                    if self.running:
                        print(f"BROKER SERIAL ERROR: {e}")
            time.sleep(0.01)

    def stop(self):
        self.running = False
        if hasattr(self, 'ser'): self.ser.close()
        sys.exit(0)

broker = RoverBroker()
signal.signal(signal.SIGINT, lambda s, f: broker.stop())

if __name__ == "__main__":
    t1 = threading.Thread(target=broker.listen_udp) # Removed daemon=True to see errors
    t2 = threading.Thread(target=broker.listen_serial)
    
    t1.start()
    t2.start()
    
    print("BROKER: Threads started. Watching for 'T' packets...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        broker.stop()