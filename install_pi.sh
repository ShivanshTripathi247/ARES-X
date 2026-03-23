#!/bin/bash
echo "🚀 Starting ARES-X Pi Installation..."

# 1. System Updates & Core Tools
sudo apt update && sudo apt upgrade -y
sudo apt install -y git python3-pip python3-colcon-common-extensions

# 2. Install Hardware Libraries (for the L298N motors)
pip install rpi-lgpio gpiozero --break-system-packages

# 3. Install ROS 2 Jazzy specific packages for Mapping
# We need the LiDAR driver, SLAM Toolbox, and a Laser Odometry package (since we dropped the Pixhawk)
sudo apt install -y \
    ros-jazzy-sllidar-ros2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-rf2o-laser-odometry \
    ros-jazzy-teleop-twist-keyboard

# 4. Setup Udev Rules for the RPLiDAR (So it always connects as /dev/rplidar)
echo "Configuring USB ports for RPLiDAR..."
wget https://raw.githubusercontent.com/Slamtec/sllidar_ros2/main/scripts/create_udev_rules.sh
chmod +x create_udev_rules.sh
sudo ./create_udev_rules.sh
rm create_udev_rules.sh

echo "Installing Pi Cam V3 (libcamera) ROS 2 drivers..."
sudo apt install -y ros-jazzy-camera-ros

echo "✅ ARES-X Pi Setup Complete! Please reboot."