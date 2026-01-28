# 설치한 ROS2 패키지 목록

이 문서는 ROS1에서 ROS2로 마이그레이션 과정에서 설치한 모든 패키지를 기록합니다.

## 기본 환경

- **OS**: Ubuntu 22.04 (또는 호환 버전)
- **ROS 버전**: ROS2 Humble
- **Python**: Python 3.10+
- **빌드 도구**: colcon

## 필수 ROS2 패키지

### 1. 메시지 패키지

#### ackermann_msgs
```bash
sudo apt install ros-humble-ackermann-msgs
```
- **용도**: Ackermann 조향 메시지 타입
- **사용 위치**:
  - `arduino_driver`: 조향 명령 수신
  - `decision`: 조향 명령 발행
- **주요 메시지 타입**: `ackermann_msgs/msg/AckermannDrive`

### 2. 이미지 처리 패키지

#### cv_bridge
```bash
sudo apt install ros-humble-cv-bridge
```
- **용도**: OpenCV와 ROS2 메시지 간 변환
- **사용 위치**:
  - `perception_pkg`: 모든 카메라 이미지 처리 노드
  - `lane_tracking_node`
  - `obstacle_detection_node`
  - `lane_marking_node`

#### image_transport
```bash
sudo apt install ros-humble-image-transport
```
- **용도**: 이미지 전송 최적화
- **사용 위치**:
  - `perception_pkg`: compressed image 지원
  - 네트워크 대역폭 최적화

### 3. 센서 드라이버

#### rplidar_ros2
```bash
sudo apt install ros-humble-rplidar-ros
```
- **용도**: RPLiDAR A1/A2/A3 센서 드라이버
- **사용 위치**:
  - `rplidar_driver`: 래퍼 패키지로 통합
  - LiDAR 장애물 감지

#### usb_cam
```bash
# ROS2에는 기본 제공, 별도 설치 불필요
# 필요시: sudo apt install ros-humble-usb-cam
```
- **용도**: USB 카메라 드라이버
- **사용 위치**:
  - `usb_cam_driver`: 래퍼 패키지로 통합
  - 차선 및 장애물 인식

## 개발 도구 패키지

### 빌드 시스템
```bash
sudo apt install python3-colcon-common-extensions
```
- **용도**: colcon 빌드 도구
- **필수 여부**: 필수

### 추가 유틸리티
```bash
sudo apt install ros-humble-rqt*
sudo apt install ros-humble-rviz2
```
- **용도**: 디버깅 및 시각화
- **필수 여부**: 선택사항 (개발 시 권장)

## 의존성 라이브러리

### C++ 라이브러리

#### Boost
```bash
sudo apt install libboost-all-dev
```
- **용도**:
  - `arduino_driver`: Boost.Asio 시리얼 통신
  - 멀티스레딩 지원

#### Eigen3
```bash
sudo apt install libeigen3-dev
```
- **용도**:
  - `perception_pkg`: 행렬 연산 (차선 피팅)
  - `lane_marking_node`: 다항식 피팅

#### OpenCV
```bash
sudo apt install libopencv-dev
```
- **용도**:
  - `perception_pkg`: 이미지 처리
  - 차선 감지, 장애물 감지, 에지 검출

### Python 라이브러리 (Phase 8 대비)

```bash
pip3 install numpy opencv-python
```
- **용도**: Python 노드 변환 시 필요
- **설치 시점**: Phase 8 진행 전

## 설치 검증

### 1. 패키지 설치 확인

```bash
# ackermann_msgs 확인
ros2 interface show ackermann_msgs/msg/AckermannDrive

# cv_bridge 확인
ros2 pkg list | grep cv_bridge

# rplidar 확인
ros2 pkg list | grep rplidar
```

### 2. 빌드 테스트

```bash
cd ~/ros2_autonomous_cpp
source /opt/ros/humble/setup.bash
colcon build
```

**예상 결과:**
```
Summary: 8 packages finished [X.XXs]
```

### 3. 런타임 의존성 확인

```bash
# 각 패키지의 의존성 확인
rosdep check --from-paths src --ignore-src
```

## 문제 해결

### 1. ackermann_msgs를 찾을 수 없음
```bash
# 에러: Could not find a package configuration file provided by "ackermann_msgs"
sudo apt update
sudo apt install ros-humble-ackermann-msgs
```

### 2. cv_bridge 빌드 오류
```bash
# 에러: opencv2/imgcodecs.hpp: No such file or directory
sudo apt install libopencv-dev ros-humble-cv-bridge
```

### 3. rplidar 패키지 이름 오류
```bash
# 잘못된 패키지 이름: ros-humble-sllidar-ros2 (존재하지 않음)
# 올바른 패키지 이름: ros-humble-rplidar-ros
sudo apt install ros-humble-rplidar-ros
```

## 패키지 버전 정보

변환 시점 기준 설치된 패키지 버전:

| 패키지 | 버전 | 설치일 |
|--------|------|--------|
| ros-humble-desktop | 0.10.0-1 | 2026-01-29 |
| ros-humble-ackermann-msgs | 2.0.2-3 | 2026-01-29 |
| ros-humble-cv-bridge | 3.2.1-2 | 2026-01-29 |
| ros-humble-image-transport | 3.1.5-3 | 2026-01-29 |
| ros-humble-rplidar-ros | 2.0.0-2 | 2026-01-29 |

## 추가 설치 예정 패키지 (Phase 8)

Python 노드 변환 시 필요한 패키지:

```bash
# YOLO 관련
pip3 install ultralytics

# TensorFlow/PyTorch (필요시)
pip3 install torch torchvision

# ROS2 Python 클라이언트 라이브러리
sudo apt install python3-rclpy
```

## 전체 설치 스크립트

한 번에 모든 필수 패키지를 설치하는 스크립트:

```bash
#!/bin/bash

echo "ROS2 Humble 자율주행 패키지 설치 시작..."

# ROS2 패키지
sudo apt update
sudo apt install -y \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rplidar-ros \
    ros-humble-rqt* \
    ros-humble-rviz2

# 의존성 라이브러리
sudo apt install -y \
    libboost-all-dev \
    libeigen3-dev \
    libopencv-dev \
    python3-colcon-common-extensions

echo "설치 완료!"
echo "다음 명령어로 빌드를 진행하세요:"
echo "  cd ~/ros2_autonomous_cpp"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build"
```

저장 위치: `/home/deepblue/ros2_autonomous_cpp/scripts/install_dependencies.sh`

## 참고 사항

1. **네트워크 환경**: 패키지 설치 시 안정적인 인터넷 연결 필요
2. **디스크 공간**: 최소 5GB 이상의 여유 공간 권장
3. **권한**: sudo 권한이 필요한 설치 항목 있음
4. **업데이트**: ROS2 Humble 패키지는 정기적으로 업데이트됨
