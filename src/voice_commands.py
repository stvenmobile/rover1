import os
import time
import socket
import json
import subprocess
import logging
import signal
from vosk import Model, KaldiRecognizer
import pyaudio

# --- LOGGING CONFIGURATION ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("rover_voice.log"),
        logging.StreamHandler()
    ]
)

# --- CONFIGURATION ---
BROKER_IP = '127.0.0.1'
UDP_CMD_PORT = 5005   # Sending commands to Broker
UDP_TELE_PORT = 5006  # Receiving telemetry from Broker

# Paths (Adjust if your models are in a different subfolder)
MODEL_PATH = "vosk-model-small-en-us" 
PIPER_MODEL = "en_US-lessac-medium.onnx" 

# Calibration Constants
TICKS_FOR_90_DEG = 850 

# --- INITIALIZE UDP SOCKET ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# SO_REUSEPORT allows us to snoop with tcpdump/other scripts simultaneously
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind(('', UDP_TELE_PORT)) 
sock.settimeout(0.1)          

def speak(text):
    """Generates speech in the background using the plughw wrapper."""
    logging.info(f"Speaking: {text}")
    # Using Popen (Background) so the logic loop continues immediately
    cmd = f"echo '{text}' | piper --model {PIPER_MODEL} --output_raw | aplay -D plughw:0,0 -r 22050 -f S16_LE -t raw"
    subprocess.Popen(cmd, shell=True)

def send_move(left, right):
    """Sends motor speed commands to the Broker on port 5005."""
    payload = {"source": "voice", "cmd": f"S{left},{right}"}
    sock.sendto(json.dumps(payload).encode(), (BROKER_IP, UDP_CMD_PORT))

def get_telemetry():
    """Reads the LATEST JSON telemetry, discarding old buffered packets."""
    latest_data = None
    while True:
        try:
            # MSG_DONTWAIT tells the OS: "Give me what you have, don't wait for more"
            data, _ = sock.recvfrom(2048, socket.MSG_DONTWAIT)
            latest_data = json.loads(data.decode())
        except (socket.error, socket.timeout, json.JSONDecodeError):
            # No more packets in the buffer
            break
    return latest_data


def is_interrupted():
    """Checks if the joystick manual override is active."""
    tele = get_telemetry()
    if tele and tele.get("manual_active"):
        logging.warning("INTERRUPT: Manual override detected!")
        return True
    return False

def move_precise(target_ticks, l_speed, r_speed):
    # 1. FLUSH the buffer first so we get the 'now' position
    get_telemetry() 
    time.sleep(0.1) # Brief pause for a fresh packet to arrive
    
    start_tele = get_telemetry()
    if not start_tele:
        logging.error("MOVE ERROR: No fresh telemetry.")
        return False
    
    start_l, start_r = start_tele['l'], start_tele['r']
    current_dist = 0
    start_time = time.time()
    
    send_move(l_speed, r_speed)
    
    while current_dist < target_ticks:
        if (time.time() - start_time) > 4.0: break
        if is_interrupted(): return False 
        
        tele = get_telemetry()
        if tele:
            # Distance calculation using the delta formula:
            # $$current\_dist = \frac{|tele_l - start_l| + |tele_r - start_r|}{2}$$
            dist_l = abs(tele['l'] - start_l)
            dist_r = abs(tele['r'] - start_r)
            current_dist = (dist_l + dist_r) / 2
        time.sleep(0.01)
        
    send_move(0, 0)
    # 2. IMPORTANT: Give the motors a moment to actually stop moving
    time.sleep(0.5) 
    return True


def run_square_precise():
    """Executes a square movement pattern (Edge -> Stop -> Turn)."""
    for i in range(4):
        if is_interrupted(): return
        speak(f"Moving edge {i+1}")
        
        # 1. Drive forward for 1.5 seconds (timed)
        send_move(180, 180)
        drive_start = time.time()
        while time.time() - drive_start < 1.5:
            if is_interrupted(): return
            time.sleep(0.05)
        
        send_move(0, 0)
        time.sleep(0.5)

        # 2. Turn 90 degrees (encoder-based)
        if is_interrupted(): return
        speak("Turning.")
        if not move_precise(TICKS_FOR_90_DEG, 130, -130):
            speak("Movement error.")
            return
        time.sleep(0.5)
        
    speak("Square sequence complete.")

def run_shutdown():
    """Gracefully shuts down all rover scripts."""
    speak("Shutting down autonomic systems. Goodbye.")
    send_move(0, 0)
    time.sleep(2.0) 
    os.system("pkill -SIGINT -f start_rover.sh")

# --- VOSK SPEECH ENGINE SETUP ---
if not os.path.exists(MODEL_PATH):
    logging.error(f"Vosk model not found at {MODEL_PATH}")
    exit(1)

model = Model(MODEL_PATH)
rec = KaldiRecognizer(model, 16000)
p = pyaudio.PyAudio()

# Using Index 0 (USB PnP Device)
mic_stream = p.open(
    format=pyaudio.paInt16, channels=1, rate=16000, 
    input=True, frames_per_buffer=8000, input_device_index=0
)
mic_stream.start_stream()

logging.info("Piper Voice System: ONLINE and LISTENING...")
speak("Piper is online.")

while True:
    data = mic_stream.read(4000, exception_on_overflow=False)
    if rec.AcceptWaveform(data):
        result = json.loads(rec.Result())
        text = result.get('text', '')
        
        if text:
            logging.info(f"Heard: '{text}'")
        
        # TRIGGER: Listen for "Piper" or common mishearings
        if any(word in text for word in ["piper", "paper", "viper"]):
            
            if "hello" in text:
                speak("Hello maker.")
                
            elif "square" in text:
                speak("Initiating square movement.")
                # Brief pause for speech to start, then move
                time.sleep(0.5) 
                run_square_precise()
                
            elif "shutdown" in text or "shut down" in text:
                run_shutdown()
                
            elif "status" in text:
                speak("Telemetry is active. Systems nominal.")
