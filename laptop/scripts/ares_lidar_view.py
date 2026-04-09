#!/usr/bin/env python3
"""
ARES-X LiDAR Visualizer
Connects to Pi LiDAR TCP stream and shows live 2D scan map
Run: python3 ares_lidar_view.py --ip 172.16.61.124
"""

import socket
import json
import math
import argparse
import threading
import time
import cv2
import numpy as np

TCP_PORT   = 8767
WIDTH      = 700
HEIGHT     = 700
CENTER_X   = WIDTH  // 2
CENTER_Y   = HEIGHT // 2
MAX_DIST   = 3000    # mm — max range to display
SCALE      = (min(WIDTH, HEIGHT) // 2 - 30) / MAX_DIST

latest_points = []
points_lock   = threading.Lock()
running       = True

def receiver(ip):
    global latest_points, running
    while running:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ip, TCP_PORT))
            print(f"Connected to LiDAR stream at {ip}:{TCP_PORT}")
            buf = ""
            while running:
                data = sock.recv(4096).decode(errors='ignore')
                if not data:
                    break
                buf += data
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    try:
                        pkt = json.loads(line)
                        if pkt.get("type") == "scan":
                            with points_lock:
                                latest_points = pkt["points"]
                    except json.JSONDecodeError:
                        pass
        except Exception as e:
            print(f"Connection lost: {e} — retrying in 2s")
            time.sleep(2)
        finally:
            try:
                sock.close()
            except Exception:
                pass

def draw_map(points):
    # Dark background
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    frame[:] = (15, 15, 15)

    # Grid rings
    for r_mm in [500, 1000, 1500, 2000, 2500, 3000]:
        r_px = int(r_mm * SCALE)
        cv2.circle(frame, (CENTER_X, CENTER_Y), r_px,
                   (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(frame, f"{r_mm//1000}.{(r_mm%1000)//100}m",
                    (CENTER_X + r_px + 4, CENTER_Y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (60, 60, 60), 1)

    # Crosshairs
    cv2.line(frame, (CENTER_X, 0), (CENTER_X, HEIGHT), (35, 35, 35), 1)
    cv2.line(frame, (0, CENTER_Y), (WIDTH, CENTER_Y),  (35, 35, 35), 1)

    # LiDAR points
    for pt in points:
        angle_rad = math.radians(pt["a"])
        dist_mm   = pt["d"]
        if dist_mm > MAX_DIST:
            continue
        px = int(CENTER_X + dist_mm * SCALE * math.sin(angle_rad))
        py = int(CENTER_Y - dist_mm * SCALE * math.cos(angle_rad))
        # Colour by distance: green=close, yellow=mid, red=far
        ratio = dist_mm / MAX_DIST
        r = int(255 * min(ratio * 2, 1))
        g = int(255 * min((1 - ratio) * 2, 1))
        cv2.circle(frame, (px, py), 2, (0, g, r), -1, cv2.LINE_AA)

    # Rover marker
    cv2.circle(frame, (CENTER_X, CENTER_Y), 8,  (0, 200, 255), -1)
    cv2.circle(frame, (CENTER_X, CENTER_Y), 8,  (255, 255, 255), 1)
    cv2.line(frame,
             (CENTER_X, CENTER_Y),
             (CENTER_X, CENTER_Y - 20),
             (0, 200, 255), 2)

    # HUD
    cv2.putText(frame, f"ARES-X LiDAR  pts:{len(points)}",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (200, 200, 200), 1)
    cv2.putText(frame, "Q to quit",
                (10, HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (80, 80, 80), 1)
    return frame

def run(ip):
    global running
    threading.Thread(target=receiver, args=(ip,), daemon=True).start()
    print("LiDAR visualizer starting — press Q to quit")
    time.sleep(1)

    while True:
        with points_lock:
            pts = list(latest_points)
        frame = draw_map(pts)
        cv2.imshow("ARES-X LiDAR Map", frame)
        if cv2.waitKey(33) & 0xFF == ord('q'):
            break

    running = False
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='172.16.61.124')
    args = parser.parse_args()
    run(args.ip)
