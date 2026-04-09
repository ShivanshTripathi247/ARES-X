#!/usr/bin/env python3
"""
ARES-X Pi Bridge v2 — with sensor data parsing
Accepts WebSocket connections from laptop dashboard
Relays drive commands to ESP32 over UART
Parses sensor JSON from ESP32 and exposes via status

Run:
  source ~/aresx/venv/bin/activate
  python3 ~/aresx/bridge/ares_bridge.py
"""

import asyncio
import serial
import json
import time
import threading
import websockets
from datetime import datetime

# ─── Config ────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUD = 115200
WS_HOST     = "0.0.0.0"
WS_PORT     = 8765

# ─── Shared state ──────────────────────────────────────────
esp_serial        = None
connected_clients = set()
serial_lock       = threading.Lock()

sensor_data = {
    "thermal_ok": False,
    "obj_temp":   -999,
    "amb_temp":   -999,
    "thermal_ts": 0,
    "imu_ok":     False,
    "acc_x":      0.0,
    "acc_y":      0.0,
    "acc_z":      0.0,
    "gyr_x":      0.0,
    "gyr_y":      0.0,
    "gyr_z":      0.0,
    "imu_ts":     0,
    "env_ok":     False,
    "pressure":   0.0,
    "altitude":   0.0,
    "mq_raw":     0,
    "mq_alert":   False,
    "env_ts":     0,
}
sensor_lock = threading.Lock()

def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")

# ─── Serial setup ───────────────────────────────────────────
def init_serial():
    global esp_serial
    try:
        esp_serial = serial.Serial(
            port          = SERIAL_PORT,
            baudrate      = SERIAL_BAUD,
            timeout       = 0.1,
            write_timeout = 1.0
        )
        log(f"Serial open: {SERIAL_PORT} @ {SERIAL_BAUD}")
        return True
    except serial.SerialException as e:
        log(f"Serial FAILED: {e}")
        return False

def send_to_esp(cmd: str):
    if esp_serial is None or not esp_serial.is_open:
        return "ERROR:NO_SERIAL"
    try:
        with serial_lock:
            esp_serial.write(cmd.encode())
            esp_serial.flush()
            time.sleep(0.02)
            if esp_serial.in_waiting:
                ack = esp_serial.readline().decode(errors='ignore').strip()
                return ack
        return "SENT"
    except serial.SerialException as e:
        return f"ERROR:{e}"

# ─── Sensor JSON parser ─────────────────────────────────────
def parse_sensor_line(line: str):
    try:
        d   = json.loads(line)
        t   = d.get("type", "")
        now = time.time()

        with sensor_lock:
            if t == "thermal":
                sensor_data["thermal_ok"] = True
                sensor_data["obj_temp"]   = d.get("object",  -999)
                sensor_data["amb_temp"]   = d.get("ambient", -999)
                sensor_data["thermal_ts"] = now
                log(f"Thermal → obj:{sensor_data['obj_temp']}°C  amb:{sensor_data['amb_temp']}°C")

            elif t == "imu":
                sensor_data["imu_ok"] = True
                sensor_data["acc_x"]  = d.get("ax", 0)
                sensor_data["acc_y"]  = d.get("ay", 0)
                sensor_data["acc_z"]  = d.get("az", 0)
                sensor_data["gyr_x"]  = d.get("gx", 0)
                sensor_data["gyr_y"]  = d.get("gy", 0)
                sensor_data["gyr_z"]  = d.get("gz", 0)
                sensor_data["imu_ts"] = now
                log(f"IMU → acc:({sensor_data['acc_x']:.1f},{sensor_data['acc_y']:.1f},{sensor_data['acc_z']:.1f})")

            elif t == "env":
                sensor_data["env_ok"]   = True
                sensor_data["pressure"] = d.get("pressure", 0)
                sensor_data["altitude"] = d.get("altitude", 0)
                sensor_data["mq_raw"]   = d.get("mq_raw",   0)
                sensor_data["mq_alert"] = d.get("mq_alert", False)
                sensor_data["env_ts"]   = now
                log(f"Env → {sensor_data['pressure']}hPa  {sensor_data['altitude']}m  AQ:{sensor_data['mq_raw']}")

    except (json.JSONDecodeError, KeyError):
        pass

# ─── Serial reader thread ───────────────────────────────────
def serial_reader():
    buf = ""
    while True:
        try:
            if esp_serial and esp_serial.is_open and esp_serial.in_waiting:
                raw  = esp_serial.read(esp_serial.in_waiting).decode(errors='ignore')
                buf += raw
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith('{'):
                        parse_sensor_line(line)
        except Exception:
            pass
        time.sleep(0.02)

# ─── Status builder ─────────────────────────────────────────
def build_status():
    with sensor_lock:
        now = time.time()
        return {
            "type":        "status",
            "serial_ok":   esp_serial.is_open if esp_serial else False,
            "clients":     len(connected_clients),
            "thermal_ok":  sensor_data["thermal_ok"],
            "obj_temp":    sensor_data["obj_temp"],
            "amb_temp":    sensor_data["amb_temp"],
            "thermal_age": round(now - sensor_data["thermal_ts"], 1) if sensor_data["thermal_ts"] else -1,
            "imu_ok":      sensor_data["imu_ok"],
            "acc_x":       sensor_data["acc_x"],
            "acc_y":       sensor_data["acc_y"],
            "acc_z":       sensor_data["acc_z"],
            "gyr_x":       sensor_data["gyr_x"],
            "gyr_y":       sensor_data["gyr_y"],
            "gyr_z":       sensor_data["gyr_z"],
            "env_ok":      sensor_data["env_ok"],
            "pressure":    sensor_data["pressure"],
            "altitude":    sensor_data["altitude"],
            "mq_raw":      sensor_data["mq_raw"],
            "mq_alert":    sensor_data["mq_alert"],
        }

# ─── WebSocket handler ──────────────────────────────────────
async def handle_client(websocket):
    client_ip = websocket.remote_address[0]
    log(f"Client connected: {client_ip}")
    connected_clients.add(websocket)

    await websocket.send(json.dumps({
        "type":   "connected",
        "msg":    "ARES-X bridge online",
        "serial": esp_serial.is_open if esp_serial else False
    }))

    try:
        async for message in websocket:
            await process_message(websocket, message)
    except websockets.exceptions.ConnectionClosed:
        log(f"Client disconnected: {client_ip}")
        send_to_esp('S')
    finally:
        connected_clients.discard(websocket)

async def process_message(websocket, message):
    try:
        data = json.loads(message)
        cmd  = data.get("cmd", "").upper().strip()

        if cmd == "STATUS":
            await websocket.send(json.dumps(build_status()))
            return

        valid_cmds = {'F', 'B', 'L', 'R', 'S', 'E', 'D', 'O', 'P', 'M'}
        if cmd not in valid_cmds:
            await websocket.send(json.dumps({
                "type": "error", "msg": f"Unknown command: {cmd}"
            }))
            return

        ack = send_to_esp(cmd)
        log(f"CMD {cmd} → ESP32 | ack: {ack}")
        await websocket.send(json.dumps({
            "type": "ack", "cmd": cmd, "esp": ack
        }))

    except json.JSONDecodeError:
        await websocket.send(json.dumps({
            "type": "error", "msg": "Invalid JSON"
        }))

# ─── Sensor broadcast task ──────────────────────────────────
async def broadcast_sensors():
    while True:
        await asyncio.sleep(1)
        if not connected_clients:
            continue
        status      = build_status()
        status["type"] = "sensors"
        msg         = json.dumps(status)
        dead        = set()
        for ws in set(connected_clients):
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        for ws in dead:
            connected_clients.discard(ws)

# ─── Main ───────────────────────────────────────────────────
async def main():
    if not init_serial():
        log("WARNING: No serial — commands and sensor data won't reach ESP32")

    threading.Thread(target=serial_reader, daemon=True).start()

    log(f"WebSocket server starting on ws://{WS_HOST}:{WS_PORT}")
    log("Waiting for laptop connection...")

    async with websockets.serve(handle_client, WS_HOST, WS_PORT):
        await asyncio.gather(
            asyncio.Future(),
            broadcast_sensors()
        )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log("Shutting down — sending stop to ESP32")
        send_to_esp('S')