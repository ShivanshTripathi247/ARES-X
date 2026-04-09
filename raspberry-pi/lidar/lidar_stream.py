#!/usr/bin/env python3
"""
ARES-X LiDAR Streamer
Reads RPLiDAR A1 scans and streams data over TCP to laptop
Laptop connects and receives JSON scan packets

Run:
  source ~/aresx/venv/bin/activate
  python3 ~/aresx/lidar/lidar_stream.py
"""

import sys
import json
import time
import socket
import threading
import logging
from rplidar import RPLidar

LIDAR_PORT  = "/dev/ttyUSB1"
TCP_HOST    = "0.0.0.0"
TCP_PORT    = 8767
MIN_QUALITY = 10       # ignore low-quality readings
MAX_CLIENTS = 3

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger("lidar")

# Shared scan data
latest_scan  = []
scan_lock    = threading.Lock()
clients      = []
clients_lock = threading.Lock()

def lidar_reader():
    """Reads scans from LiDAR and updates latest_scan"""
    global latest_scan
    while True:
        try:
             # Flush stale bytes before connecting — fixes descriptor mismatch
            import serial
            with serial.Serial(LIDAR_PORT, 115200, timeout=1) as s:
                s.reset_input_buffer()
                s.reset_output_buffer()

            import time as _t
            _t.sleep(0.5)  # let motor spin up cleanly

            lidar = RPLidar(LIDAR_PORT)
            info  = lidar.get_info()
            log.info(f"LiDAR connected — model {info['model']} firmware {info['firmware']}")
            lidar.start_motor()

            for scan in lidar.iter_scans(max_buf_meas=500):
                points = [
                    {"q": q, "a": round(a, 2), "d": round(d, 1)}
                    for q, a, d in scan
                    if q >= MIN_QUALITY and d > 0
                ]
                with scan_lock:
                    latest_scan = points

        except Exception as e:
            log.warning(f"LiDAR error: {e} — retrying in 2s")
            time.sleep(2)
        finally:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass

def broadcast_scans():
    """Sends latest scan to all connected clients at ~10Hz"""
    while True:
        time.sleep(0.1)
        with scan_lock:
            if not latest_scan:
                continue
            packet = json.dumps({
                "type":   "scan",
                "points": latest_scan,
                "ts":     time.time()
            }) + "\n"

        dead = []
        with clients_lock:
            for conn in clients:
                try:
                    conn.sendall(packet.encode())
                except Exception:
                    dead.append(conn)
            for conn in dead:
                clients.remove(conn)
                log.info("Client disconnected")

def handle_client(conn, addr):
    log.info(f"Client connected: {addr[0]}")
    with clients_lock:
        clients.append(conn)
    # Keep thread alive — broadcast_scans handles sending
    try:
        while True:
            data = conn.recv(64)
            if not data:
                break
    except Exception:
        pass
    finally:
        with clients_lock:
            if conn in clients:
                clients.remove(conn)
        conn.close()
        log.info(f"Client gone: {addr[0]}")

def tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_HOST, TCP_PORT))
    srv.listen(MAX_CLIENTS)
    log.info(f"LiDAR TCP server on port {TCP_PORT}")
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=handle_client,
                         args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    threading.Thread(target=lidar_reader,  daemon=True).start()
    threading.Thread(target=broadcast_scans, daemon=True).start()
    log.info("Waiting for scan data...")
    time.sleep(2)
    try:
        tcp_server()
    except KeyboardInterrupt:
        log.info("Shutting down")
