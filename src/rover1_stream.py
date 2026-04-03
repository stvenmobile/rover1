import cv2
from flask import Flask, Response

app = Flask(__name__)

# 1. Define the GStreamer pipeline FIRST
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
    )

# 2. Initialize the Raspberry Pi Camera 2.1
print("--- Starting GStreamer Pipeline for Rover1 ---")
pipeline = gstreamer_pipeline(flip_method=0)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

def gen_frames():
    while True:
        success, frame = cap.read()
        if not success:
            print("Failed to capture frame. Is the ribbon cable secure?")
            break
        
        # This is where your 1,024 CUDA cores live! 
        # For now, we're just streaming clean BGR frames.
        
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>Rover1 Live Feed</h1><img src='/video_feed' width='640'>"

if __name__ == '__main__':
    try:
        # Host on all interfaces (WiFi: .131, Ethernet: .130)
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        cap.release()
        print("Camera released cleanly.")
