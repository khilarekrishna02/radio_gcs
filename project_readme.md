# Drone Intrusion Detection & Real-Time Geotagging Pipeline

This repository contains the software payload for an autonomous drone designed to detect, track, and geotag human targets using an NVIDIA Jetson, Intel RealSense camera, and Pixhawk flight controller. 

## 🚀 Hardware Stack
*   **Compute:** NVIDIA Jetson (GPU-accelerated via CUDA)
*   **Vision:** Intel RealSense Depth Camera (1280x720 @ 30FPS for minimal motion blur)
*   **Flight Controller:** Pixhawk (MAVLink Telemetry & Lidar)

## 🧠 Model Weights
Due to GitHub size constraints, the YOLOv8 model weights (`bestbest.pt`) are stored externally. 
**Download the weights here:** `[PASTE YOUR GOOGLE DRIVE LINK HERE]`
Place the downloaded `.pt` file in the root directory before running any scripts.

## 🏗️ The 3-Tier Testing Pipeline
This architecture is split into three progressive stages to ensure safe and accurate deployment:

1.  **Offline Visual Tracker:** Evaluates the YOLOv8 model's raw tracking stability and confidence thresholds (locked at 0.50) using pre-recorded `.mp4` files.
2.  **Offline Math Simulator:** Validates the 3D projective geometry, Ray-Intersect math, and CSV logging by pairing pre-recorded video with simulated Pixhawk telemetry.
3.  **Live Jetson Flight Payload (`live_jetson_geotagger.py`):** The final production script. It integrates live RealSense camera feeds, exact millisecond time-syncing, RANSAC spatial clustering (to merge duplicate ID coordinates), and transmits the final filtered GPS coordinates to the drone via pure MAVLink telemetry.

## ⚙️ Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/YOUR-USERNAME/YOUR-REPO-NAME.git
   cd YOUR-REPO-NAME
   ```
2. Install the required dependencies:
   ```bash
   pip3 install -r requirements.txt
   ```
3. Download the model weights from the link above.

## 🚁 Running the Live Payload
Ensure your RealSense and Pixhawk are connected via USB, then execute:
```bash
python3 live_jetson_geotagger.py
```
*Note: Make sure your Pixhawk port matches the `PIXHAWK_PORT` variable in the script (default: `/dev/ttyACM0`).*