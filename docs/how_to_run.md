# ROS2 자율주행 시스템 실행 가이드

Docker 컨테이너 환경에서 ROS2 자율주행 시스템을 실행하는 방법을 설명합니다.

---

## 빠른 시작 (Quick Start)

```bash
# 1. 호스트에서 X11 권한 설정
xhost +local:docker

# 2. Docker 컨테이너 시작
cd /home/deepblue/target_projects/adas_env
docker compose up -d

# 3. 컨테이너 접속
docker exec -it adas_container bash

# 4. ROS2 빌드 (처음 한 번만)
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 5. 전체 시스템 실행
ros2 launch bringup track_launch.py

# 또는 테스트 모드 (센서 없이 모터 테스트)
ros2 launch bringup track_launch.py test_mode:=true
```

---

## 목차

1. [사전 준비](#사전-준비)
2. [Docker 환경 설정](#docker-환경-설정)
3. [컨테이너 실행](#컨테이너-실행)
4. [ROS2 빌드](#ros2-빌드)
5. [개별 테스트](#개별-테스트)
6. [전체 시스템 실행](#전체-시스템-실행)
7. [모니터링 및 시각화](#모니터링-및-시각화)
8. [문제 해결](#문제-해결)

---


## 사전 준비

### 하드웨어 연결

| 장치 | 연결 포트 | 설명 |
|------|-----------|------|
| Arduino | `/dev/ttyACM0` | 모터/서보 제어 |
| RPLiDAR | `/dev/ttyUSB0` | 장애물 감지 |
| USB 웹캠 | `/dev/video4` | 차선 인식 (Logitech C920) |


### 장치 확인

```bash
# 연결된 장치 확인
ls /dev/ttyACM*   # Arduino
ls /dev/ttyUSB*   # LiDAR
ls /dev/video*    # Camera

# 카메라 장치 상세 확인
v4l2-ctl --list-devices
```

---

## Docker 환경 설정

### 파일 위치

```
/home/deepblue/target_projects/adas_env/
├── Dockerfile        # 커스텀 이미지 정의
└── compose.yaml      # 컨테이너 설정
```

### Dockerfile

```dockerfile
FROM osrf/ros:humble-desktop

# 필수 ROS2 패키지 및 pip 설치
RUN apt update && apt install -y \
    ros-humble-usb-cam \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    v4l-utils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python 패키지 설치
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python \
    pyserial

# ROS2 환경 자동 설정
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

WORKDIR /root/ros2_ws
```

### compose.yaml

```yaml
services:
  adas-dev:
    build: .
    image: adas-ros2:humble
    container_name: adas_container
    network_mode: host
    ipc: host
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /home/deepblue/target_projects/ros2_autonomous_cpp/ros2_autonomous_cpp:/root/ros2_ws
    environment:
      - DISPLAY=${DISPLAY}
    stdin_open: true
    tty: true
    privileged: true
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
      - /dev/ttyUSB0:/dev/ttyUSB0
      - /dev/video0:/dev/video0
      - /dev/video4:/dev/video4
```

---

## 컨테이너 실행

### 1. X11 권한 설정 (호스트에서)

```bash
xhost +local:docker
```

### 2. 이미지 빌드 (처음 한 번만)

```bash
cd /home/deepblue/target_projects/adas_env
docker compose build
```

> **참고:** Dockerfile이 변경되면 다시 빌드 필요

### 3. 컨테이너 시작

```bash
docker compose up -d
```

### 4. 컨테이너 접속

```bash
docker exec -it adas_container bash
```

### 5. ROS2 환경 설정 (컨테이너 내부)

Dockerfile에서 `.bashrc`에 자동 설정했으므로, 새 터미널에서는 자동 적용됩니다.

첫 빌드 후에는 수동으로 한 번 실행:
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

---

## ROS2 빌드

### 필수 패키지

Dockerfile에 이미 포함되어 있어 **별도 설치 불필요**:
- `ros-humble-usb-cam`
- `ros-humble-ackermann-msgs`
- `ros-humble-cv-bridge`
- `ros-humble-image-transport`
- `numpy`, `opencv-python`, `pyserial`

### 빌드

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 빌드 캐시 충돌 시

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## 개별 테스트

### 카메라 테스트

**터미널 1 - 카메라 노드 실행:**
```bash
ros2 launch usb_cam_driver usb_cam_launch.py
```

**터미널 2 - 확인:**
```bash
# 토픽 확인
ros2 topic list | grep camera

# 발행 속도 확인
ros2 topic hz /camera/front/image

# 이미지 뷰어
ros2 run rqt_image_view rqt_image_view
# → /camera/front/image 선택
```

### Arduino 모터 테스트

**터미널 1 - Arduino 노드:**
```bash
ros2 run arduino_driver arduino_bridge_node --ros-args \
  --params-file /root/ros2_ws/install/arduino_driver/share/arduino_driver/config/arduino.yaml
```

**터미널 2 - 수동 명령:**
```bash
# 전진 (속도 0.5 m/s)
ros2 topic pub --once /cmd_vel ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.5, steering_angle: 0.0}}"

# 좌회전
ros2 topic pub --once /cmd_vel ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.3, steering_angle: 0.26}}"

# 정지
ros2 topic pub --once /cmd_vel ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.0}}"
```

**긴급 정지 (시리얼 직접):**
```bash
echo "S" > /dev/ttyACM0
```

### Arduino 명령 모드

현재 설정: **연속 모드** (`use_legacy_cmd: false`)

| 모드 | 설정값 | 명령 형식 | 설명 |
|------|--------|-----------|------|
| 레거시 | `true` | `F`, `B`, `L`, `R`, `S` | 단순 방향 명령 |
| 연속 | `false` | `V:pwm,S:servo` | PWM/서보 값 직접 전달 |

**연속 모드 장점:**
- 더 정밀한 속도 제어 (0~255 PWM)
- 더 정밀한 조향 제어 (서보 각도 직접)
- 불필요한 명령 변환 없음

**설정 변경 방법:**
```yaml
# src/drivers/arduino_driver/config/arduino.yaml
use_legacy_cmd: false  # 연속 모드 (권장)
```

---

## 전체 시스템 실행

### Track 모드 (차선 추종)

```bash
ros2 launch bringup track_launch.py
```

**실행되는 노드:**
- `usb_cam_node_exe` - 카메라
- `lane_tracking_node` - 차선 추적
- `lane_marking_node` - 차선 표시 검출
- `arduino_bridge_node` - 모터 제어
- `ultrasonic_processor_node` - 초음파 센서
- `lidar_obstacle_node` - 라이다 장애물
- `decision_node` - 주행 결정

### 옵션라이

```bash
# Python 노드 사용 (C++ 대신)
ros2 launch bringup track_launch.py use_cpp:=false

# AI 결정 모드
ros2 launch bringup track_launch.py decision_mode:=ai

# 테스트 모드 (센서 없이 모터 구동)
ros2 launch bringup track_launch.py test_mode:=true
```

### 테스트 모드 설명

`test_mode:=true` 옵션을 사용하면 센서 데이터 없이도 모터가 구동됩니다.

**우회되는 체크:**
- LiDAR 장애물 감지
- 초음파 안전 거리 (0.2m)
- 신호등 상태
- 차선 데이터 타임아웃

**주의사항:**
- 테스트 모드에서는 안전 기능이 비활성화되므로 주의 필요
- 실제 주행 전 반드시 `test_mode:=false`로 변경
- 모터 동작 확인 및 튜닝 목적으로만 사용

---

## 모니터링 및 시각화

### 토픽 목록

```bash
ros2 topic list
```

### 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/front/image` | Image | 카메라 원본 영상 |
| `/lane_overlay` | Image | 차선 인식 시각화 |
| `/lane/steering_angle` | Float32 | 차선 기반 조향 각도 |
| `/lane/center_offset` | Float32 | 차선 중심 오프셋 |
| `/arduino/cmd` | AckermannDriveStamped | 최종 모터 명령 |
| `/cmd_vel` | AckermannDriveStamped | 속도/조향 명령 |

### 토픽 모니터링

```bash
# 조향 각도 실시간 확인
ros2 topic echo /lane/steering_angle

# 모터 명령 확인
ros2 topic echo /arduino/cmd

# 발행 속도 확인
ros2 topic hz /lane/steering_angle
```

### 이미지 시각화

```bash
ros2 run rqt_image_view rqt_image_view
```

**선택 가능한 토픽:**
- `/camera/front/image` - 원본 카메라 영상
- `/lane_overlay` - 차선 인식 오버레이

### 전체 시스템 모니터링

```bash
rqt
# → Plugins > Topics > Topic Monitor
# → Plugins > Visualization > Image View
```

---

## 문제 해결

### 노트북 내장 카메라가 켜질 때

USB 웹캠 장치 번호 확인:
```bash
v4l2-ctl --list-devices
```

config 파일 수정 (`usb_cam.yaml`, `usb_cam_launch.py`):
```yaml
video_device: "/dev/video4"  # USB 웹캠 번호로 변경
```

### 토픽이 발행되지 않을 때

```bash
# 노드 상태 확인
ros2 node list

# 특정 토픽 상태 확인
ros2 topic info /camera/front/image
```

### 컨테이너 재시작

```bash
cd /home/deepblue/target_projects/adas_env
docker compose down
docker compose up -d
```

### 차선 인식이 안 될 때

1. 카메라 영상 확인
2. 조명 조건 확인 (너무 밝거나 어두우면 안 됨)
3. 흰색 선이 카메라에 보이는지 확인
4. 파라미터 조정 (`lane_params.yaml`):
   ```yaml
   canny_low: 50      # 낮추면 더 민감
   canny_high: 150    # 낮추면 더 민감
   roi_y_ratio: 0.55  # 관심 영역 비율
   ```

---

## 멀티 터미널 사용

여러 터미널에서 동시에 작업할 수 있습니다:

**터미널 1 (호스트):**
```bash
xhost +local:docker
```

**터미널 2, 3, 4... (각각 컨테이너 접속):**
```bash
docker exec -it adas_container bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

---

## 테스트 체크리스트

- [ ] Docker 이미지 빌드 (`docker compose build`)
- [ ] Docker 컨테이너 실행 (`docker compose up -d`)
- [ ] X11 권한 설정 (`xhost +local:docker`)
- [ ] ROS2 워크스페이스 빌드 (`colcon build --symlink-install`)
- [ ] 카메라 영상 확인 (`rqt_image_view`)
- [ ] 차선 인식 확인 (`/lane_overlay`)
- [ ] 모터 수동 테스트 (`ros2 topic pub`)
- [ ] 전체 시스템 테스트 (`track_launch.py`)

---

## 컨테이너 재접속 vs 재시작

| 상황 | 명령어 | 설치한 패키지 | ROS2 환경 |
|------|--------|--------------|-----------|
| 터미널 닫고 다시 접속 | `docker exec -it adas_container bash` | 유지됨 | 자동 설정 |
| 컨테이너 재시작 | `docker compose down && docker compose up -d` | **Dockerfile에 포함되어 유지** | 자동 설정 |
| 이미지 재빌드 | `docker compose build` | 새로 설치 | 자동 설정 |


스크립트 생성 완료. 사용법:


cd /home/deepblue/target_projects/adas_env

./start.sh              # 시작 + 접속 (가장 많이 사용)
./start.sh restart      # 재시작 (장치 새로 꽂은 후)
./start.sh build        # 이미지 빌드 후 시작 (처음 또는 Dockerfile 변경 시)
./start.sh stop         # 컨테이너 정지
실행하면 자동으로:

xhost +local:docker (X11 권한)
연결 장치 확인 (Arduino, LiDAR, Camera)
컨테이너 시작
컨테이너 내부 장치 마운트 확인
컨테이너 접속
지금 USB 웹캠 마운트 문제를 해결하려면:


./start.sh restart