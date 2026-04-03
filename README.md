# rover1: AI-Enhanced Mobile Robotics Platform 🤖

**rover1** is an advanced autonomous platform designed for high-performance edge computing, computer vision, and natural language interaction.

---

## 🏗️ Core System Architecture

### 🧠 High-Level Controller (HLC)
* **Compute:** NVIDIA Jetson Orin Nano 8GB Developer Kit
* **Architecture:** Ampere (1024 CUDA Cores, 32 Tensor Cores)
* **Environment:** JetPack 6.2.1 (L4T 36.4.7) / Ubuntu 22.04 LTS
* **Thermals:** Stable at **48-49°C** during active vision/teleop tasks.

### 🛡️ Low-Level Controller (LLC)
* **Compute:** Maker ESP32 Pro (NULLLAB)
* **Diagnostics:** Integrated **0.96" OLED Dashboard** (Address 0x3C) providing real-time state feedback:
    * **STANDBY:** Initial state; awaiting first command.
    * **ACTIVE:** Valid communication link with HLC established.
    * **TIMEOUT!:** Safety Watchdog engaged (2s pulse loss).
* **Focus:** Interrupt-driven quadrature encoder tracking and PWM motor drive.

---

## ⚡ Power & Performance Benchmarks
* **Regulation:** 19V Buck-Boost converter ensuring steady power for Orin transient spikes.
* **Idle Consumption:** ~5W (Camera stream + Teleop active).
* **Load Consumption:** 7.2W - 7.4W (Full-speed drive on carpet transitions).
* **Mobility:** High-torque configuration optimized for navigating floor-to-carpet transitions.

---

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