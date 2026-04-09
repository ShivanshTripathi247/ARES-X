#!/usr/bin/env python3
"""
ARES-X Rear Camera Streamer — Pi Camera v3 (imx708) on CAM1
Serves MJPEG on port 8768
Run: python3 ~/aresx/camera/rear_stream.py
"""

import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')

import io
import time
import threading
import logging
from http.server import BaseHTTPRequestHandler, HTTPServer
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

HOST   = "0.0.0.0"
PORT   = 8768
WIDTH  = 640
HEIGHT = 480
FPS    = 30

logging.basicConfig(level=logging.INFO,
                    format='[%(asctime)s] %(message)s',
                    datefmt='%H:%M:%S')
log = logging.getLogger("rear_cam")

class StreamBuffer(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.lock  = threading.Condition()

    def write(self, data):
        with self.lock:
            self.frame = bytes(data)
            self.lock.notify_all()
        return len(data)

    def read_frame(self, timeout=2.0):
        with self.lock:
            self.lock.wait(timeout)
            return self.frame

buffer = StreamBuffer()

class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            log.info(f"Rear client: {self.client_address[0]}")
            try:
                while True:
                    frame = buffer.read_frame(timeout=2.0)
                    if frame is None:
                        continue
                    self.wfile.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(frame)).encode() + b'\r\n'
                        b'\r\n' + frame + b'\r\n'
                    )
            except (BrokenPipeError, ConnectionResetError):
                log.info("Rear client disconnected")

        elif self.path == '/health':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'OK')
        else:
            self.send_response(404)
            self.end_headers()

def run_camera():
    # Camera 1 = imx708 (Pi Camera v3), rotation=180 to correct mounting
    cam = Picamera2(1)
    config = cam.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": FPS}
    )
    cam.configure(config)
    log.info(f"Rear camera: {cam.camera_properties['Model']} "
             f"@ {WIDTH}x{HEIGHT} {FPS}fps")

    encoder = MJPEGEncoder(10000000)
    output  = FileOutput(buffer)
    cam.start_recording(encoder, output)
    log.info("Rear camera recording started")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop_recording()
        cam.close()

if __name__ == "__main__":
    threading.Thread(target=run_camera, daemon=True).start()
    time.sleep(2)
    server = HTTPServer((HOST, PORT), StreamHandler)
    log.info(f"Rear stream: http://<PI_IP>:{PORT}/stream")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
