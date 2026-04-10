# rover: AI-Enhanced Mobile Robotics Platform 🤖

**rover** is an advanced autonomous platform designed for high-performance edge computing, computer vision, and natural language interaction.

---

## 🏗️ Core System Architecture

### 🧠 High-Level Controller (HLC)
* **Compute:** NVIDIA Jetson Orin Nano 8GB Developer Kit
* **Environment:** JetPack 6.2.1 / Ubuntu 22.04 LTS
* **Role:** Vision processing, Speech-to-Text (Vosk), and Neural TTS (Piper).

### 🛡️ Low-Level Controller (LLC)
* **Compute:** Maker ESP32 Pro (NULLLAB)
* **Role:** Interrupt-driven encoder tracking and PWM motor drive.
* **Safety:** Autonomic Nervous System (ANS) for cliff and obstacle detection.

---

## 📡 Communication Protocol (Serial @ 115200)

| Direction | Format | Description |
| :--- | :--- | :--- |
| **Jetson -> ESP32** | `S[L],[R]\n` | Set Motor PWM (-255 to 255) |
| **ESP32 -> Jetson** | `E[L],[R],[S],[C]\n` | Ticks, Safety State, Cliff Distance |

---

## 👁️ Computer Vision & Audio
* **Vision:** GStreamer pipeline (IMX219) -> Flask FPV stream.
* **Voice:** Vosk (STT) and Piper (TTS) for local intent processing.
* **Mux:** TCA9548A I2C Multiplexer for 5x VL53L1X ToF and BNO085 IMU.

## ⚡ Power & Performance Benchmarks
* **Regulation:** 19V Buck-Boost converter ensuring steady power for Orin transient spikes.
* **Idle Consumption:** ~5W (Camera stream + Teleop active).
* **Load Consumption:** 7.2W - 7.4W (Full-speed drive on carpet transitions).
* **Mobility:** High-torque configuration optimized for navigating floor-to-carpet transitions.

---

## 📡 Communication Protocol (v2 - ANS Enabled)

| Direction | Format | Description |
| :--- | :--- | :--- |
| **Jetson -> ESP32** | `S[L],[R]\n` | Set Motor PWM (-255 to 255) |
| **ESP32 -> Jetson** | `E[L],[R],[S],[C]\n` | Ticks, Safety State (0/1), Cliff Dist |

---

### 🛠️ I2C Multiplexer (TCA9548A) Mapping
The LLC utilizes a multiplexer to isolate high-density sensor clusters:

* **CH0:** BNO085 (9-DOF IMU)
* **CH1:** VL53L1X (Front Center)
* **CH2:** VL53L1X (Front Left - 22.5°)
* **CH3:** VL53L1X (Front Right - 22.5°)
* **CH4:** VL53L1X (Front Downward - Cliff Detection)
* **CH5:** VL53L1X (Rear Center)
* **CH6:** RGB LED Controller (Placeholder)

### 🧠 Autonomic Nervous System (ANS)
The LLC maintains an autonomous safety loop that operates independently of HLC commands:
1. [cite_start]**Cliff Detection:** Emergency stop if `CH4` distance exceeds 120mm. [cite: 23, 28]
2. [cite_start]**Obstacle Avoidance:** Hard stop if any forward ToF sensor detects an object < 150mm. [cite: 23]
3. [cite_start]**Heartbeat:** 2000ms watchdog for Jetson serial link loss. [cite: 7, 23]

## 📡 Communication Protocol (Serial @ 115200)

The Jetson Orin Nano delegates real-time tasks to the ESP32 via a strict heartbeat protocol.

| Direction | Format | Description |
| :--- | :--- | :--- |
| **Jetson -> ESP32** | `S[L],[R]\n` | Set Motor PWM (-255 to 255) |
| **ESP32 -> Jetson** | `E[L],[R]\n` | Signed Encoder Ticks (10Hz Telemetry) |

> [!IMPORTANT]
> **Safety Watchdog:** The LLC will execute an emergency hardware stop if no valid `S` command is received within a **2000ms** window.

---

## 👁️ Computer Vision (GStreamer)
The platform utilizes a hardware-accelerated GStreamer pipeline to feed the **Flask FPV stream**:
* **Source:** Raspberry Pi Camera 2.1 (IMX219)
* **Pipeline:** `nvarguscamerasrc` -> `nvvidconv` -> `videoconvert`
* **Resolution:** 1280x720 capture / 640x360 stream @ 30fps.

---


## 🛠️ Key Hardware Specification
* **Actuators:** 2x JGB37 DC Motors with Hall Effect Encoders
* **Sensors:** 4x VL53L1X Time-of-Flight (ToF) sensors & BNO085 9-DOF IMU
* **Audio:** Waveshare USB-to-I2S Audio Card & Dual Microphones
* **Compute:** Jetson Orin Nano 8GB (HLC) & Maker ESP32 Pro (LLC)

---

## 🗣️ Speech & Intent
* **STT (Ears):** **Vosk** (Offline, lightweight speech-to-text)
* **TTS (Voice):** **Piper** (Local, high-quality neural voice synthesis)
* **Intent:** Flexible natural language processing for varied command structures.

## 🗣️ Speech Intent Library (Vosk/Piper) 

| Wake Word | Command | Action | LLC Execution |
| :--- | :--- | :--- | :--- |
| **Rover** | "Hello" | TTS: "Hi maker, how are you?" | N/A |
| **Rover** | "Square" | Performs a 4-edge square path | `S150,150` (1s) -> `S100,-100` (Turn) |


---