FROM osrf/ros:humble-desktop

# 필수 ROS2 패키지 및 pip 설치
RUN apt update && apt install -y \
    ros-humble-usb-cam \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rplidar-ros \
    v4l-utils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python 패키지 설치
RUN pip3 install --no-cache-dir \
    "numpy<2" \
    opencv-python \
    pyserial \
    ultralytics

# ROS2 환경 자동 설정
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

WORKDIR /root/ros2_ws
