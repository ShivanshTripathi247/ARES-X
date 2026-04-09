#!/usr/bin/env python3
"""
ARES-X YOLO Inference
Connects to Pi MJPEG stream, runs YOLOv8n, shows annotated feed
Run: python3 ares_yolo.py --ip 172.16.61.124
"""

import cv2
import argparse
import urllib.request
import numpy as np
from ultralytics import YOLO

PI_CAM_PORT = 8766

def run(ip):
    stream_url = f"http://{ip}:{PI_CAM_PORT}/stream"
    print(f"Loading YOLOv8n model...")
    model = YOLO("yolo26m.pt")
    print(f"Connecting to stream: {stream_url}")

    stream = urllib.request.urlopen(stream_url, timeout=10)
    buf   = b""

    print("Stream connected — press Q to quit")

    while True:
        buf += stream.read(4096)

        # Find JPEG frame boundaries
        start = buf.find(b'\xff\xd8')
        end   = buf.find(b'\xff\xd9')

        if start == -1 or end == -1:
            continue

        jpg   = buf[start:end+2]
        buf   = buf[end+2:]

        # Decode frame
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue

        # Run YOLO inference
        results = model(frame, verbose=False, conf=0.4)
        annotated = results[0].plot()

        # Show FPS and detection count
        n = len(results[0].boxes)
        cv2.putText(annotated, f"ARES-X | Objects: {n}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("ARES-X YOLO Feed", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    print("Done.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='172.16.61.124', help='Pi IP address')
    args = parser.parse_args()
    run(args.ip)
