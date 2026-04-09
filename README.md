# ARES-X — Autonomous Robotic Exploration System

A six-wheel autonomous ground rover with real-time AI vision, 360° LiDAR mapping, and multi-modal sensing. Built without ROS 2 — pure Python WebSocket/TCP stack.

## Hardware
- Raspberry Pi 5 (8GB) + ESP32
- RPLiDAR A1M8, Pi NoIR Camera, Pi Camera v3
- MLX90614 thermal, MPU6500 IMU, BMP280, MQ135
- L298N motor driver, 6-wheel chassis, 11.1V LiPo

## Repository Structure
laptop/          — FastAPI dashboard, YOLO inference, segmentation (runs on operator laptop)

raspberry-pi/    — Bridge, camera streamers, LiDAR relay (runs on Pi 5)

## Running

**On Pi:**
```bash
source ~/aresx/venv/bin/activate
python3 raspberry-pi/ares_bridge.py
python3 raspberry-pi/camera_stream.py
python3 raspberry-pi/rear_stream.py
python3 raspberry-pi/lidar_stream.py
```

**On Laptop:**
```bash
source ~/aresx-laptop/venv/bin/activate
python3 laptop/ares_dashboard.py \
  --pi <PI_TAILSCALE_IP> \
  --model yolo26m.pt \
  --seg yolov8s-seg.pt \
  --yolop YOLOP/weights/End-to-end.pth
```

## Tech Stack
- ESP32: Arduino C++ (motor control, sensors, NeoPixel, UART bridge)
- Pi: Python 3.11, Picamera2, rplidar, websockets, pyserial
- Laptop: Python 3.10, FastAPI, Ultralytics YOLOv8, YOLOP, OpenCV, Chart.js
- Remote access: Tailscale VPN
