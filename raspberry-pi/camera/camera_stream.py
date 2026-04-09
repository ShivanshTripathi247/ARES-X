#!/usr/bin/env python3
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
PORT   = 8766
WIDTH  = 640
HEIGHT = 480
FPS    = 30

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(message)s', datefmt='%H:%M:%S')
log = logging.getLogger("camera")

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
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            log.info(f"Client connected: {self.client_address[0]}")
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
                log.info(f"Client disconnected: {self.client_address[0]}")

        elif self.path == '/snapshot':
            frame = buffer.read_frame(timeout=2.0)
            if frame:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(frame)
            else:
                self.send_response(503)
                self.end_headers()

        elif self.path == '/health':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'OK')

        else:
            self.send_response(404)
            self.end_headers()

def run_camera():
    cam = Picamera2()
    config = cam.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": FPS}
    )
    cam.configure(config)
    log.info(f"Camera: {cam.camera_properties['Model']} @ {WIDTH}x{HEIGHT} {FPS}fps")
    encoder = MJPEGEncoder(10000000)
    output  = FileOutput(buffer)
    cam.start_recording(encoder, output)
    log.info("Recording started")
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
    log.info(f"Stream: http://172.16.61.124:{PORT}/stream")
    log.info(f"Snapshot: http://172.16.61.124:{PORT}/snapshot")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
