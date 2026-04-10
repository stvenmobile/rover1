#!/bin/bash

# --- ROVER IGNITION SCRIPT ---
# Location: ~/rover/start_rover.sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
SRC_DIR="$DIR/src"
LOG_DIR="$DIR/logs"
mkdir -p "$LOG_DIR"

echo "------------------------------------------"
echo "🚀 Initializing Rover Autonomic Systems..."
echo "------------------------------------------"

# THE FIX: Change directory to src so relative paths work
cd "$SRC_DIR"

# 1. Start the Serial Broker
# Note: Since we are already in src, we just call the filename
python3 main.py > "$LOG_DIR/broker.log" 2>&1 &
BROKER_PID=$!
sleep 3

# 2. Start the GStreamer Vision Stream
python3 rover_stream.py > "$LOG_DIR/stream.log" 2>&1 &
STREAM_PID=$!
echo "✅ Rover Stream is active at HTTP://192.168.1.131:5000."

# 3. Start Voice Commands
python3 voice_commands.py > "$LOG_DIR/voice.log" 2>&1 &
VOICE_PID=$!

echo "✅ Systems active. Monitor voice with: tail -f ../logs/voice.log"

cleanup() {
    echo -e "\n🛑 Shutting down Rover..."
    # Killing processes by name to ensure absolute cleanup
    pkill -f main.py
    pkill -f rover_stream.py
    pkill -f voice_commands.py
    exit
}

trap cleanup SIGINT
wait
