FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-cv-bridge \
    ros-jazzy-fastrtps \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages ultralytics opencv-python

WORKDIR /ares_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ares_ws/ares_pc_ws/install/setup.bash" >> ~/.bashrc