#!/bin/bash
# ARES-X Laptop Setup Script
# Run once on your Pop!_OS / Ubuntu laptop
# Usage: bash setup_laptop.sh

set -e

echo "======================================"
echo "  ARES-X Laptop Setup"
echo "======================================"

# ─── Check Python ──────────────────────────────────────────
echo "[1/4] Checking Python..."
python3 --version || { echo "Python3 not found"; exit 1; }

# ─── Create project folder ─────────────────────────────────
echo "[2/4] Creating laptop project folder..."
mkdir -p ~/aresx-laptop/{dashboard,scripts,models,logs}

cat > ~/aresx-laptop/README.md << 'EOF'
ARES-X Laptop Side
===================
dashboard/   Web dashboard (FastAPI server)
scripts/     Keyboard controller, test scripts
models/      YOLO model weights
logs/        Runtime logs
EOF

# ─── Virtual environment ───────────────────────────────────
echo "[3/4] Setting up Python virtual environment..."
python3 -m venv ~/aresx-laptop/venv
source ~/aresx-laptop/venv/bin/activate
pip install --upgrade pip
pip install -r requirements-laptop.txt

# ─── Download YOLO model ───────────────────────────────────
echo "[4/4] Downloading YOLOv8n model..."
python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')  # downloads automatically on first run
print('YOLOv8n model ready:', model.info())
" 2>/dev/null || echo "YOLO model will download on first run"

# Move model to models folder if downloaded
if [ -f "yolov8n.pt" ]; then
    mv yolov8n.pt ~/aresx-laptop/models/
    echo "Model saved to ~/aresx-laptop/models/yolov8n.pt"
fi

deactivate

# ─── Done ──────────────────────────────────────────────────
echo ""
echo "======================================"
echo "  Laptop setup complete!"
echo ""
echo "  To activate:"
echo "  source ~/aresx-laptop/venv/bin/activate"
echo ""
echo "  To run keyboard controller:"
echo "  python3 scripts/ares_keyboard.py --ip <PI_IP>"
echo "======================================"