# Startup Sequence

## Raspberry Pi Terminal Commands

### Terminal 1
```
source ~/aresx/venv/bin/activate
python3 ~/aresx/camera/camera_stream.py
```

### Terminal 2
```
source ~/aresx/venv/bin/activate
python3 ~/aresx/camera/rear_stream.py
```
### Terminal 3
```
source ~/aresx/venv/bin/activate
python3 ~/aresx/bridge/ares_bridge.py
```
### Terminal 4
```
source ~/aresx/venv/bin/activate
python3 ~/aresx/lidar/lidar_stream.py
```
-------
## Laptop Terminal Command
```
source /home/stargazer_unix/aresx-laptop/venv/bin/activate
python3 dashboard/ares_dashboard.py   --pi 10.216.155.250(YOUR PI's IP ADDRESS)   --model yolo26m.pt   --seg yolov8s-seg.pt   --yolop YOLOP/weights/End-to-end.pth
```