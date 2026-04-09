# ─── ARES-X Central Config ───────────────────────────────────
# Edit this file to change IPs, ports, pins — nothing else needs touching

# Network
PI_HOST         = "0.0.0.0"       # Pi listens on all interfaces
LAPTOP_IP       = ""               # set this once you know laptop IP
WS_PORT         = 8765             # WebSocket — bridge
CAM_PORT        = 8766             # HTTP — MJPEG stream
LIDAR_PORT      = 8767             # TCP — LiDAR scan data

# Serial (Pi <-> ESP32)
SERIAL_PORT     = "/dev/ttyAMA0"
SERIAL_BAUD     = 115200

# Camera
CAM_WIDTH       = 640
CAM_HEIGHT      = 480
CAM_FPS         = 30
CAM_DEVICE      = 0                # /dev/video0

# LiDAR
LIDAR_PORT_DEV  = "/dev/ttyUSB0"   # RPLiDAR on USB
LIDAR_BAUD      = 115200

# Safety gate
SAFETY_DIST_CM  = 25.0
CONFIRM_COUNT   = 3
