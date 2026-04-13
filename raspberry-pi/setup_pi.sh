#!/bin/bash
# ARES-X Pi Setup Script
# Run once on a fresh Raspberry Pi OS Bookworm Lite 64-bit
# Usage: bash setup_pi.sh

set -e  # exit on any error

echo "======================================"
echo "  ARES-X Pi Setup"
echo "======================================"

# ─── Locale fix ────────────────────────────────────────────
echo "[1/7] Fixing locale..."
sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
export LC_ALL=en_GB.UTF-8
export LANG=en_GB.UTF-8

# ─── System updates ────────────────────────────────────────
echo "[2/7] Updating system packages..."
sudo apt update && sudo apt upgrade -y

# ─── System dependencies ───────────────────────────────────
echo "[3/7] Installing system dependencies..."
sudo apt install -y \
    python3-pip \
    python3-venv \
    python3-libcamera \
    python3-picamera2 \
    python3-opencv \
    libcap-dev \
    git \
    v4l-utils \
    i2c-tools

# ─── Enable interfaces via raspi-config ────────────────────
echo "[4/7] Configuring camera, UART, I2C..."
# Dual camera setup for Pi 5
sudo tee -a /boot/firmware/config.txt << 'CONF'

# ARES-X — dual camera config
camera_auto_detect=0
dtoverlay=ov5647,cam0
dtoverlay=imx708,cam1

# Enable UART for ESP32 bridge
enable_uart=1
CONF

# Enable I2C
sudo raspi-config nonint do_i2c 0

# Disable serial console (keep hardware UART free for ESP32)
sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true

# ─── Project folder structure ──────────────────────────────
echo "[5/7] Creating project structure..."
mkdir -p ~/aresx/{bridge,camera,lidar,utils,logs,config}

cat > ~/aresx/README.md << 'EOF'
ARES-X Rover — Project Structure
=================================
bridge/      Pi <-> ESP32 UART + WebSocket server
camera/      Camera capture and MJPEG streaming
lidar/       RPLiDAR reader and data forwarder
dashboard/   Laptop-side UI (copy to laptop)
utils/       Shared helpers
logs/        Runtime logs
config/      All configuration in one place
EOF

cat > ~/aresx/config/settings.py << 'EOF'
# ─── ARES-X Central Config ───────────────────────────────────
# Edit this file to change IPs, ports — nothing else needs touching

# Network
PI_HOST         = "0.0.0.0"
LAPTOP_IP       = ""               # set this to your laptop IP
WS_PORT         = 8765             # WebSocket bridge
CAM_PORT        = 8766             # MJPEG stream
LIDAR_PORT      = 8767             # LiDAR TCP socket

# Serial (Pi <-> ESP32)
SERIAL_PORT     = "/dev/ttyAMA0"
SERIAL_BAUD     = 115200

# Camera
CAM_WIDTH       = 640
CAM_HEIGHT      = 480
CAM_FPS         = 30

# LiDAR
LIDAR_PORT_DEV  = "/dev/ttyUSB0"
LIDAR_BAUD      = 115200

# Safety
SAFETY_DIST_CM  = 25.0
CONFIRM_COUNT   = 3
EOF

# ─── Python venv ───────────────────────────────────────────
echo "[6/7] Setting up Python virtual environment..."
python3 -m venv ~/aresx/venv --system-site-packages
source ~/aresx/venv/bin/activate
pip install --upgrade pip
pip install -r ~/aresx/requirements-pi.txt
deactivate

# ─── Auto-activate venv on SSH login ───────────────────────
echo "[7/7] Configuring shell..."
# Add venv activation to .bashrc if not already there
if ! grep -q "aresx/venv" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ARES-X — auto activate venv" >> ~/.bashrc
    echo "source ~/aresx/venv/bin/activate" >> ~/.bashrc
    echo "cd ~/aresx" >> ~/.bashrc
fi

# ─── Serial port permissions ───────────────────────────────
sudo usermod -aG dialout $USER
sudo usermod -aG tty $USER

# ─── Done ──────────────────────────────────────────────────
echo ""
echo "======================================"
echo "  Setup complete!"
echo "  REBOOT REQUIRED for all changes"
echo "  to take effect."
echo ""
echo "  After reboot:"
echo "  1. SSH back in"
echo "  2. venv will auto-activate"
echo "  3. Copy your scripts to ~/aresx/"
echo "======================================"
echo ""
read -p "Reboot now? (y/n): " choice
if [ "$choice" = "y" ]; then
    sudo reboot
fi