import cv2
import socket
import json
import threading
import time
from flask import Flask, Response, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)
app.config['SECRET_KEY'] = 'rover_secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# --- TELEMETRY STORAGE ---
latest_data = {
    "tofs": [0, 0, 0, 0, 0], 
    "imu": [0.0, 0.0, 0.0, 0.0], 
    "ticks": [0, 0],
    "last_seen": 0
}

def telemetry_listener():
    """Listens for UDP telemetry from main.py."""
    global latest_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', 5006)) 
    except Exception as e:
        print(f"UDP BIND ERROR: {e}")
        return

    while True:
        try:
            data, _ = sock.recvfrom(2048)
            packet = json.loads(data.decode())
            packet["last_seen"] = time.time()
            latest_data = packet
            socketio.emit('tele_update', latest_data)
        except:
            time.sleep(0.1)

threading.Thread(target=telemetry_listener, daemon=True).start()

# --- GSTREAMER PIPELINE ---
def gstreamer_pipeline(sensor_id=0, flip_method=0):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=640, height=360, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=True max-buffers=1 wait-on-eos=false" 
        % (sensor_id, flip_method)
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

def gen_frames():
    while True:
        success, frame = cap.read()
        if not success or frame is None:
            time.sleep(0.01)
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Rover1 Dashboard</title>
        <script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
        <style>
            body { font-family: 'Courier New', monospace; background: #000; color: #0f0; text-align: center; margin: 0; }
            .container { padding: 10px; }
            img { border: 2px solid #0f0; max-width: 640px; width: 100%; height: auto; }
            .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 640px; margin: 15px auto; }
            .box { background: #111; border: 1px solid #333; padding: 10px; border-radius: 4px; }
            .val { font-size: 1.4em; color: #fff; }
            .label { font-size: 0.7em; color: #0f0; margin-bottom: 5px; }
            #status { padding: 5px; font-size: 0.8em; }
            #runtime { color: #888; font-size: 0.9em; margin-top: 5px; }
            .stale { color: #f00; }
            .active { color: #0f0; }
        </style>
    </head>
    <body>
        <div class="container">
            <h2>ROVER1 DIAGNOSTIC</h2>
            <img src="/video_feed">
            <div id="status" class="stale">--- WAITING FOR BROKER ---</div>
            <div id="runtime">Session Time: 00:00</div>

            <div class="grid">
                <div class="box"><div class="label">FRONT LEFT</div><div id="tof1" class="val">--</div></div>
                <div class="box"><div class="label">FRONT CENTER</div><div id="tof0" class="val">--</div></div>
                <div class="box"><div class="label">FRONT RIGHT</div><div id="tof2" class="val">--</div></div>
                <div class="box"><div class="label">REAR</div><div id="tof4" class="val">--</div></div>
                <div class="box"><div class="label">DOWNWARD</div><div id="tof3" class="val">--</div></div>
                <div class="box"><div class="label">STABILITY (W)</div><div id="imu" class="val">--</div></div>
            </div>
        </div>

        <script>
            // Use local-ish SocketIO
            var socket = io();
            var lastUpdate = 0;
            var startTime = Date.now();
            
            socket.on('tele_update', function(data) {
                lastUpdate = Date.now();
                document.getElementById('tof0').innerText = data.tofs[0] + "mm";
                document.getElementById('tof1').innerText = data.tofs[1] + "mm";
                document.getElementById('tof2').innerText = data.tofs[2] + "mm";
                document.getElementById('tof3').innerText = data.tofs[3] + "mm";
                document.getElementById('tof4').innerText = data.tofs[4] + "mm";
                document.getElementById('imu').innerText = data.imu[0].toFixed(2);
                
                var status = document.getElementById('status');
                status.innerText = "STREAM ACTIVE @ " + new Date().toLocaleTimeString();
                status.className = "active";
            });

            // Timer Loop
            setInterval(function() {
                var elapsed = Math.floor((Date.now() - startTime) / 1000);
                var mins = String(Math.floor(elapsed / 60)).padStart(2, '0');
                var secs = String(elapsed % 60).padStart(2, '0');
                document.getElementById('runtime').innerText = "Session Time: " + mins + ":" + secs;

                if (lastUpdate > 0 && (Date.now() - lastUpdate > 3000)) {
                    document.getElementById('status').innerText = "BROKER DATA STALE";
                    document.getElementById('status').className = "stale";
                }
            }, 1000);
        </script>
    </body>
    </html>
    """)

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)