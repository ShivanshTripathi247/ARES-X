#!/usr/bin/env python3
"""
ARES-X Unified Dashboard v5
- Data logging: detections → CSV, sensors → JSON (~/aresx-laptop/logs/)
- Live scrolling charts: temperature, IMU magnitude, air quality, pressure
- Mode buttons: obstacle avoid, follow, manual
- Shared camera reader (single pipe to Pi)

Run:
  cd ~/aresx-laptop
  python3 dashboard/ares_dashboard.py \
    --pi 172.16.61.124 \
    --model yolo26m.pt \
    --seg "/home/stargazer_unix/segmentation model/yolo26m-seg.pt" \
    --yolop YOLOP/weights/End-to-end.pth
"""

import argparse
import asyncio
import csv
import json
import os
import sys
import socket
import threading
import time
import logging
import uvicorn
import websockets
import cv2
import numpy as np
import urllib.request
from datetime import datetime
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from ultralytics import YOLO

logging.basicConfig(level=logging.INFO,
                    format='[%(asctime)s] %(message)s',
                    datefmt='%H:%M:%S')
log = logging.getLogger("dashboard")

# ─── Args ──────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--pi',    default='172.16.61.124')
parser.add_argument('--port',  default=9000, type=int)
parser.add_argument('--model', default='yolo26m.pt')
parser.add_argument('--seg',   default='yolov8s-seg.pt')
parser.add_argument('--yolop', default='YOLOP/weights/End-to-end.pth')
parser.add_argument('--conf',  default=0.4, type=float)
parser.add_argument('--logs',  default='logs', help='Log directory')
args, _ = parser.parse_known_args()

PI_IP     = args.pi
DASH_PORT = args.port
WS_PI     = f"ws://{PI_IP}:8765"
CAM_URL   = f"http://{PI_IP}:8766/stream"
REAR_URL  = f"http://{PI_IP}:8768/stream"
LIDAR_TCP = (PI_IP, 8767)

# ═══════════════════════════════════════════════════════════
# DATA LOGGING
# ═══════════════════════════════════════════════════════════
LOG_DIR = Path(args.logs)
LOG_DIR.mkdir(parents=True, exist_ok=True)

session_start = datetime.now()
date_str      = session_start.strftime('%Y-%m-%d')

DETECTION_CSV = LOG_DIR / f"detections_{date_str}.csv"
SENSOR_JSON   = LOG_DIR / f"sensors_{date_str}.jsonl"
SESSION_LOG   = LOG_DIR / "sessions.log"

# Init detection CSV header if new file
if not DETECTION_CSV.exists():
    with open(DETECTION_CSV, 'w', newline='') as f:
        csv.writer(f).writerow([
            'timestamp', 'class', 'confidence', 'x1', 'y1', 'x2', 'y2'
        ])

# Log session start
with open(SESSION_LOG, 'a') as f:
    f.write(f"[{session_start.isoformat()}] Session started\n")

log.info(f"Logging detections → {DETECTION_CSV}")
log.info(f"Logging sensors    → {SENSOR_JSON}")

det_log_lock    = threading.Lock()
sensor_log_lock = threading.Lock()
session_stats   = {"detections": 0, "frames": 0, "start": time.time()}

def log_detections(results, names):
    """Write each detection to CSV."""
    ts = datetime.now().isoformat()
    rows = []
    for box in results[0].boxes:
        cls  = int(box.cls[0])
        conf = float(box.conf[0])
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
        rows.append([ts, names[cls], f"{conf:.3f}", x1, y1, x2, y2])
    if rows:
        with det_log_lock:
            with open(DETECTION_CSV, 'a', newline='') as f:
                csv.writer(f).writerows(rows)
        session_stats["detections"] += len(rows)

def log_sensor_snapshot(data: dict):
    """Append sensor snapshot to JSONL file."""
    record = {"ts": datetime.now().isoformat(), **data}
    with sensor_log_lock:
        with open(SENSOR_JSON, 'a') as f:
            f.write(json.dumps(record) + '\n')

last_sensor_log = 0

# ═══════════════════════════════════════════════════════════
# MODELS
# ═══════════════════════════════════════════════════════════
log.info(f"Loading detection model: {args.model}")
try:
    det_model = YOLO(args.model)
    log.info("Detection model ready")
except Exception as e:
    log.error(f"Detection model failed: {e}")
    det_model = None

seg_mode       = 'indoor'
seg_mode_lock  = threading.Lock()
seg_model      = None
seg_model_lock = threading.Lock()
seg_switching  = False

def load_seg_model(mode: str):
    global seg_model, seg_switching
    seg_switching = True
    time.sleep(0.2)
    log.info(f"Loading seg model: {mode}")
    try:
        with seg_model_lock:
            seg_model = None
        if mode == 'indoor':
            m = YOLO(args.seg)
            with seg_model_lock:
                seg_model = ('indoor', m)
            log.info("Indoor seg ready")
        elif mode == 'outdoor':
            if not os.path.exists(args.yolop):
                log.warning(f"YOLOP weights not found: {args.yolop}")
                return
            yolop_dir = os.path.dirname(os.path.dirname(os.path.abspath(args.yolop)))
            if yolop_dir not in sys.path:
                sys.path.insert(0, yolop_dir)
            import torch
            from lib.config import cfg
            from lib.models import get_net
            device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
            m = get_net(cfg)
            ckpt = torch.load(args.yolop, map_location=device, weights_only=False)
            m.load_state_dict(ckpt['state_dict'])
            m = m.to(device).eval()
            with seg_model_lock:
                seg_model = ('outdoor', m, device)
            log.info("Outdoor seg ready (YOLOP)")
    except Exception as e:
        log.error(f"Seg model load failed: {e}")
    finally:
        seg_switching = False

threading.Thread(target=load_seg_model, args=('indoor',), daemon=True).start()

# ═══════════════════════════════════════════════════════════
# SHARED CAMERA READER — one pipe, two consumers
# ═══════════════════════════════════════════════════════════
raw_frame       = None
raw_frame_lock  = threading.Lock()
raw_frame_event = threading.Event()

def camera_reader_thread():
    global raw_frame
    while True:
        try:
            stream = urllib.request.urlopen(CAM_URL, timeout=10)
            log.info("Camera reader connected")
            buf = b""
            while True:
                buf += stream.read(4096)
                s    = buf.find(b'\xff\xd8')
                e    = buf.find(b'\xff\xd9')
                if s == -1 or e == -1:
                    continue
                jpg = buf[s:e+2]
                buf = buf[e+2:]
                frame = cv2.imdecode(
                    np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                with raw_frame_lock:
                    raw_frame = frame
                raw_frame_event.set()
                raw_frame_event.clear()
                session_stats["frames"] += 1
        except Exception as e:
            log.warning(f"Camera reader error: {e} — retry 2s")
            with raw_frame_lock:
                raw_frame = None
            time.sleep(2)

threading.Thread(target=camera_reader_thread, daemon=True).start()

# ─── Output frame buffers ──────────────────────────────────
det_frame       = None
det_lock        = threading.Lock()
seg_frame       = None
seg_lock        = threading.Lock()
rear_frame      = None
rear_lock       = threading.Lock()
detection_count = 0
inference_fps   = 0.0

def yolo_detection_thread():
    global det_frame, detection_count, inference_fps
    t_last = time.time(); frame_count = 0
    names  = det_model.names if det_model else {}
    while True:
        raw_frame_event.wait(timeout=2.0)
        with raw_frame_lock:
            frame = raw_frame.copy() if raw_frame is not None else None
        if frame is None:
            time.sleep(0.05); continue
        try:
            if det_model:
                results   = det_model(frame, verbose=False, conf=args.conf)
                annotated = results[0].plot()
                n_det     = len(results[0].boxes)
                # Log detections
                if n_det > 0:
                    log_detections(results, names)
            else:
                annotated = frame.copy()
                n_det     = 0
            frame_count += 1
            elapsed = time.time() - t_last
            if elapsed >= 1.0:
                inference_fps = round(frame_count / elapsed, 1)
                frame_count   = 0
                t_last        = time.time()
            _, jpeg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with det_lock:
                det_frame       = jpeg.tobytes()
                detection_count = n_det
        except Exception as e:
            log.warning(f"Detection error: {e}")

threading.Thread(target=yolo_detection_thread, daemon=True).start()

def seg_indoor(frame):
    with seg_model_lock:
        if seg_model is None or seg_model[0] != 'indoor': return None
        model = seg_model[1]
    h, w = frame.shape[:2]
    results  = model(frame, verbose=False, conf=0.35)
    obstacle = np.zeros((h, w), dtype=np.uint8)
    if results[0].masks is not None:
        for mask in results[0].masks.data:
            m = cv2.resize(mask.cpu().numpy(), (w, h))
            obstacle = cv2.bitwise_or(obstacle, (m > 0.5).astype(np.uint8))
    free = np.zeros((h, w), dtype=np.uint8)
    free[int(h*0.4):, :] = 1
    free = cv2.bitwise_and(free, cv2.bitwise_not(obstacle))
    out = frame.copy()
    out[free==1]     = (out[free==1]     * 0.45 + np.array([0,180,0])  * 0.55).astype(np.uint8)
    out[obstacle==1] = (out[obstacle==1] * 0.50 + np.array([0,0,200]) * 0.50).astype(np.uint8)
    ann   = results[0].plot(masks=False, boxes=True)
    final = cv2.addWeighted(out, 0.75, ann, 0.25, 0)
    pct   = int(free.sum() / max(w*h*0.6, 1) * 100)
    cv2.putText(final, f'INDOOR  Clear:{pct}%', (8,20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,150), 1)
    return final

def seg_outdoor(frame):
    import torch, torchvision.transforms as T, PIL.Image as PILImage
    with seg_model_lock:
        if seg_model is None or seg_model[0] != 'outdoor': return None
        _, model, device = seg_model
    h, w = frame.shape[:2]
    transform = T.Compose([T.ToTensor(),
        T.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])])
    img_rgb = cv2.cvtColor(cv2.resize(frame,(640,640)), cv2.COLOR_BGR2RGB)
    inp = transform(PILImage.fromarray(img_rgb)).unsqueeze(0).to(device)
    half = device.type != 'cpu'
    if half: model.half(); inp = inp.half()
    else: inp = inp.float()
    with torch.no_grad():
        _, da_out, ll_out = model(inp)
    da = torch.nn.functional.interpolate(da_out, size=(h,w), mode='bilinear', align_corners=False)
    _, da = torch.max(da, 1); da = da.squeeze().cpu().numpy().astype(np.uint8)
    ll = torch.nn.functional.interpolate(ll_out, size=(h,w), mode='bilinear', align_corners=False)
    _, ll = torch.max(ll, 1); ll = ll.squeeze().cpu().numpy().astype(np.uint8)
    out = frame.copy()
    out[da==1] = (out[da==1]*0.5 + np.array([0,180,0]) *0.5).astype(np.uint8)
    out[ll==1] = (out[ll==1]*0.4 + np.array([0,0,220]) *0.6).astype(np.uint8)
    cv2.putText(out,'OUTDOOR — YOLOP',(8,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,150),1)
    return out

def segmentation_thread():
    global seg_frame
    while True:
        raw_frame_event.wait(timeout=2.0)
        with raw_frame_lock:
            frame = raw_frame.copy() if raw_frame is not None else None
        if frame is None or seg_switching:
            time.sleep(0.05); continue
        try:
            with seg_mode_lock: mode = seg_mode
            result = seg_indoor(frame) if mode=='indoor' else seg_outdoor(frame)
            if result is None: continue
            _, jpeg = cv2.imencode('.jpg', result, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with seg_lock: seg_frame = jpeg.tobytes()
        except Exception as e:
            log.warning(f"Seg error: {e}")

threading.Thread(target=segmentation_thread, daemon=True).start()

def rear_thread():
    global rear_frame
    while True:
        try:
            stream = urllib.request.urlopen(REAR_URL, timeout=10)
            log.info("Rear stream connected")
            buf = b""
            while True:
                buf += stream.read(4096)
                s = buf.find(b'\xff\xd8'); e = buf.find(b'\xff\xd9')
                if s==-1 or e==-1: continue
                jpg = buf[s:e+2]; buf = buf[e+2:]
                frame = cv2.imdecode(np.frombuffer(jpg,dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                with rear_lock: rear_frame = jpeg.tobytes()
        except Exception as e:
            log.warning(f"Rear cam error: {e} — retry 3s")
            with rear_lock: rear_frame = None
            time.sleep(3)

threading.Thread(target=rear_thread, daemon=True).start()

# ─── LiDAR ─────────────────────────────────────────────────
latest_scan = []
scan_lock   = threading.Lock()

def lidar_receiver():
    global latest_scan
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5); sock.connect(LIDAR_TCP)
            buf = ""
            while True:
                chunk = sock.recv(4096).decode(errors='ignore')
                if not chunk: break
                buf += chunk
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    try:
                        pkt = json.loads(line)
                        if pkt.get("type") == "scan":
                            with scan_lock: latest_scan = pkt["points"]
                    except Exception: pass
        except Exception as e:
            log.warning(f"LiDAR: {e} — retry 2s"); time.sleep(2)
        finally:
            try: sock.close()
            except Exception: pass

threading.Thread(target=lidar_receiver, daemon=True).start()

# ─── Sensor cache + history for charts ─────────────────────
sensor_cache = {
    "thermal_ok":False,"obj_temp":-999,"amb_temp":-999,
    "imu_ok":False,"acc_x":0.0,"acc_y":0.0,"acc_z":0.0,
    "gyr_x":0.0,"gyr_y":0.0,"gyr_z":0.0,
    "env_ok":False,"pressure":0.0,"altitude":0.0,
    "mq_raw":0,"mq_alert":False,"last_update":0,
}
sensor_cache_lock = threading.Lock()

# Rolling history for charts — last 60 readings (120s at 2s interval)
MAX_HISTORY = 60
sensor_history = {
    "timestamps": [],
    "obj_temp":   [],
    "imu_mag":    [],   # sqrt(ax²+ay²+az²)
    "mq_raw":     [],
    "pressure":   [],
}
history_lock = threading.Lock()

def update_history(data: dict):
    global sensor_history
    ts = datetime.now().strftime('%H:%M:%S')
    imu_mag = 0.0
    if data.get("imu_ok"):
        ax = data.get("acc_x", 0)
        ay = data.get("acc_y", 0)
        az = data.get("acc_z", 0)
        imu_mag = round((ax**2 + ay**2 + az**2)**0.5, 2)
    with history_lock:
        sensor_history["timestamps"].append(ts)
        sensor_history["obj_temp"].append(
            data.get("obj_temp", None) if data.get("thermal_ok") else None)
        sensor_history["imu_mag"].append(imu_mag if data.get("imu_ok") else None)
        sensor_history["mq_raw"].append(
            data.get("mq_raw", None) if data.get("env_ok") else None)
        sensor_history["pressure"].append(
            data.get("pressure", None) if data.get("env_ok") else None)
        # Trim to MAX_HISTORY
        for k in sensor_history:
            if len(sensor_history[k]) > MAX_HISTORY:
                sensor_history[k] = sensor_history[k][-MAX_HISTORY:]

async def pi_sensor_listener():
    global last_sensor_log
    while True:
        try:
            async with websockets.connect(WS_PI, ping_interval=10) as ws:
                log.info("Sensor listener connected")
                async for msg in ws:
                    try:
                        d = json.loads(msg)
                        if d.get("type") in ("sensors", "status"):
                            with sensor_cache_lock:
                                for k in sensor_cache:
                                    if k in d: sensor_cache[k] = d[k]
                                sensor_cache["last_update"] = time.time()
                            # Update chart history
                            with sensor_cache_lock:
                                snap = dict(sensor_cache)
                            update_history(snap)
                            # Log to file every 2s
                            now = time.time()
                            if now - last_sensor_log >= 2.0:
                                last_sensor_log = now
                                log_sensor_snapshot(snap)
                    except Exception: pass
        except Exception as e:
            log.warning(f"Sensor listener: {e} — retry 3s")
            await asyncio.sleep(3)

# ─── FastAPI ────────────────────────────────────────────────
app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

def make_generator(get_fn, fps=30):
    while True:
        frame = get_fn()
        if frame is None: time.sleep(0.05); continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
               + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
        time.sleep(1/fps)

@app.get("/camera")
def cam_detection():
    def get():
        with det_lock: return det_frame
    return StreamingResponse(make_generator(get),
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/camera/seg")
def cam_seg():
    def get():
        with seg_lock: return seg_frame
    return StreamingResponse(make_generator(get),
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/camera/rear")
def cam_rear():
    def get():
        with rear_lock: return rear_frame
    return StreamingResponse(make_generator(get, fps=15),
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.post("/seg/mode/{mode}")
def set_seg_mode(mode: str):
    global seg_mode
    if mode not in ('indoor','outdoor'):
        return JSONResponse({"error":"invalid mode"}, status_code=400)
    with seg_mode_lock:
        if seg_mode == mode: return {"mode":mode,"status":"unchanged"}
        seg_mode = mode
    threading.Thread(target=load_seg_model, args=(mode,), daemon=True).start()
    return {"mode":mode,"status":"switching"}

@app.get("/seg/status")
def seg_status():
    with seg_mode_lock: mode = seg_mode
    with seg_model_lock: loaded = seg_model[0] if seg_model else None
    return {"mode":mode,"loaded":loaded,"switching":seg_switching}

@app.get("/scan")
def get_scan():
    with scan_lock: return {"points": latest_scan}

@app.get("/sensors")
def get_sensors():
    with sensor_cache_lock: return dict(sensor_cache)

@app.get("/history")
def get_history():
    with history_lock: return dict(sensor_history)

@app.get("/log/stats")
def get_log_stats():
    elapsed = time.time() - session_stats["start"]
    return {
        "session_start":    session_start.isoformat(),
        "elapsed_seconds":  int(elapsed),
        "total_detections": session_stats["detections"],
        "total_frames":     session_stats["frames"],
        "detection_csv":    str(DETECTION_CSV),
        "sensor_jsonl":     str(SENSOR_JSON),
    }

pi_ws      = None
pi_ws_lock = asyncio.Lock()

def is_ws_open(ws):
    try:    return ws is not None and ws.close_code is None
    except AttributeError:
        try:    return ws is not None and not ws.closed
        except: return False

async def get_pi_ws():
    global pi_ws
    async with pi_ws_lock:
        if not is_ws_open(pi_ws):
            try:
                pi_ws = await websockets.connect(WS_PI, ping_interval=5)
                log.info(f"Pi bridge connected: {WS_PI}")
            except Exception as e:
                log.warning(f"Pi bridge unavailable: {e}")
                pi_ws = None
        return pi_ws

@app.websocket("/ws")
async def dashboard_ws(ws: WebSocket):
    await ws.accept()
    log.info("Browser connected")
    try:
        while True:
            data = await ws.receive_text()
            try:
                msg = json.loads(data)
                cmd = msg.get("cmd","").upper()
                pi  = await get_pi_ws()
                if pi:
                    await pi.send(json.dumps({"cmd":cmd}))
                    try:
                        ack = await asyncio.wait_for(pi.recv(), timeout=0.3)
                        await ws.send_text(ack)
                    except asyncio.TimeoutError:
                        await ws.send_text(json.dumps({"type":"ack","cmd":cmd,"esp":"timeout"}))
                else:
                    await ws.send_text(json.dumps({"type":"error","msg":"Pi not connected"}))
            except Exception as e:
                await ws.send_text(json.dumps({"type":"error","msg":str(e)}))
    except WebSocketDisconnect:
        log.info("Browser disconnected")
        # Log session end
        elapsed = int(time.time() - session_stats["start"])
        with open(SESSION_LOG, 'a') as f:
            f.write(f"[{datetime.now().isoformat()}] Session ended — "
                    f"{elapsed}s · {session_stats['detections']} detections · "
                    f"{session_stats['frames']} frames\n")

@app.get("/status")
async def status():
    pi = await get_pi_ws()
    with scan_lock:   lidar_ok = len(latest_scan) > 0
    with det_lock:    cam_ok   = det_frame is not None
    with seg_lock:    seg_ok   = seg_frame is not None
    with rear_lock:   rear_ok  = rear_frame is not None
    return {
        "pi_bridge":     is_ws_open(pi),
        "lidar":         lidar_ok,
        "camera":        cam_ok,
        "seg":           seg_ok,
        "rear":          rear_ok,
        "scan_pts":      len(latest_scan),
        "detections":    detection_count,
        "inference_fps": inference_fps,
        "pi_ip":         PI_IP,
    }

# ═══════════════════════════════════════════════════════════
# DASHBOARD HTML
# ═══════════════════════════════════════════════════════════
@app.get("/", response_class=HTMLResponse)
def dashboard():
    html_path = Path(__file__).parent / "index.html"
    return html_path.read_text()

@app.on_event("startup")
async def startup():
    asyncio.create_task(pi_sensor_listener())

if __name__ == "__main__":
    log.info(f"ARES-X Dashboard v5 -> http://localhost:{DASH_PORT}")
    log.info(f"Pi: {PI_IP} | Detection: {args.model} | Seg: {args.seg}")
    log.info(f"Logs: {LOG_DIR.resolve()}")
    uvicorn.run(app, host="0.0.0.0", port=DASH_PORT, log_level="warning")