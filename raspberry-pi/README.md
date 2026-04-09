ARES-X Rover — Project Structure
=================================
bridge/      Pi <-> ESP32 UART + WebSocket server
camera/      Camera capture and MJPEG streaming
lidar/       RPLiDAR reader and data forwarder
dashboard/   Laptop-side UI (copy to laptop)
utils/       Shared helpers
logs/        Runtime logs
config/      All configuration in one place
