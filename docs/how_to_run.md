# ROS2 자율주행 시스템 실행 가이드

Docker 컨테이너 환경에서 ROS2 자율주행 시스템을 실행하는 방법을 설명합니다.

---

## 빠른 시작 (Quick Start)

```bash
# 1. start.sh로 간편 실행 (권장)
cd /home/deepblue/target_projects/adas_env
./start.sh              # 컨테이너 시작 + 접속

# 2. ROS2 빌드 (처음 한 번만, 컨테이너 내부에서)
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 3. 전체 시스템 실행
ros2 launch bringup track_launch.py

# 또는 테스트 모드 (센서 없이 모터 테스트)
ros2 launch bringup track_launch.py test_mode:=true
```

**수동 방법 (start.sh 없이):**

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
```

---

## 목차

1. [사전 준비](#사전-준비)
2. [Docker 환경 설정](#docker-환경-설정)
3. [start.sh 사용법](#startsh-사용법)
4. [컨테이너 실행](#컨테이너-실행)
5. [ROS2 빌드](#ros2-빌드)
6. [개별 테스트](#개별-테스트)
7. [YOLOv8n-seg 실행](#yolov8n-seg-실행)
8. [전체 시스템 실행](#전체-시스템-실행)
9. [모니터링 및 시각화](#모니터링-및-시각화)
10. [문제 해결](#문제-해결)

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
├── compose.yaml      # 컨테이너 설정
└── start.sh          # 간편 실행 스크립트
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
    ros-humble-rplidar-ros \
    v4l-utils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python 패키지 설치 (numpy<2 필수: cv_bridge 호환성)
RUN pip3 install --no-cache-dir \
    "numpy<2" \
    opencv-python \
    pyserial \
    ultralytics

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

## start.sh 사용법

스크립트 위치: `/home/deepblue/target_projects/adas_env/start.sh`

start.sh는 Docker 컨테이너 관리 및 ROS2 실행을 간편하게 해주는 스크립트입니다.
실행하면 자동으로 xhost 권한 설정, 장치 확인, 컨테이너 시작 등을 처리합니다.

```bash
cd /home/deepblue/target_projects/adas_env
```

| 명령어 | 설명 |
|--------|------|
| `./start.sh` | 컨테이너 시작 + 접속 (가장 많이 사용) |
| `./start.sh build` | Docker 이미지 빌드 후 시작 (처음 또는 Dockerfile 변경 시) |
| `./start.sh ros-build` | ROS2 colcon 빌드 (컨테이너 내부에서 실행) |
| `./start.sh run` | track_launch.py 실행 |
| `./start.sh test` | test_mode로 실행 (센서 없이 모터 테스트) |
| `./start.sh camera` | 카메라+차선인식 테스트 (Python, 주행 없음) |
| `./start.sh camera_cpp` | 카메라+차선인식 테스트 (C++, 주행 없음) |
| `./start.sh yolo` | YOLOv8n-seg 노드 실행 |
| `./start.sh yolo-drive` | YOLO 차선 주행 모드 (세그멘테이션 기반) |
| `./start.sh rviz` | rviz2 실행 (토픽 자동 설정) |
| `./start.sh restart` | 컨테이너 재시작 (장치 재마운트, USB 장치를 새로 꽂은 후 사용) |
| `./start.sh stop` | 컨테이너 정지 |
| `./start.sh help` | 도움말 |

### 일반적인 워크플로우

```bash
# 처음 설정
./start.sh build        # 이미지 빌드 + 시작
./start.sh ros-build    # ROS2 빌드

# 일상 사용
./start.sh              # 접속
./start.sh run          # 시스템 실행

# 차선 인식 테스트 (주행 없이 카메라만)
./start.sh camera       # Python 차선 인식 (재빌드 불필요)
./start.sh camera_cpp   # C++ 차선 인식 (ros-build 필요)

# 테스트
./start.sh test         # 센서 없이 모터 테스트

# 장치 문제 시
./start.sh restart      # 컨테이너 재시작 (장치 재마운트)
```

---

## 컨테이너 실행

> **참고:** start.sh를 사용하면 아래 과정이 자동으로 처리됩니다.

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
- `ros-humble-rplidar-ros`
- `numpy`, `opencv-python`, `pyserial`, `ultralytics`

### 빌드

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

또는 start.sh 사용:
```bash
./start.sh ros-build
```

### 빌드 캐시 충돌 시

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## 개별 테스트

### 카메라 + 차선 인식 테스트 (권장)

카메라와 차선 인식만 실행하여 주행 없이 차선 인식 상태를 확인하는 모드입니다.
자동으로 카메라를 감지하고, rqt_image_view 2개 창(원본/오버레이)을 띄웁니다.

```bash
# Python 차선 인식 (재빌드 불필요, 파일 수정 즉시 반영)
./start.sh camera

# C++ 차선 인식 (colcon build 필요)
./start.sh ros-build        # C++ 변경 시 빌드 필요
./start.sh camera_cpp
```

**동작 순서:**
1. X11 권한 설정 + 컨테이너 시작
2. USB 카메라 자동 감지 (v4l2-ctl로 Video Capture 장치 확인)
3. 컨테이너 내 카메라 장치 확인 (없으면 자동 재시작)
4. (Python 모드만) numpy 버전 호환성 자동 체크
5. `camera_test_launch.py` 실행 → rqt_image_view x2 자동 실행

**시각화 화면:**
- 창 1: `/camera/front/image` - 카메라 원본 영상
- 창 2: `/lane_overlay` - 차선 인식 오버레이
  - 녹색 작은 선: 원시 Hough 세그먼트
  - 파란색 곡선: 왼쪽 차선 피팅 (RANSAC + 2차 다항식)
  - 빨간색 곡선: 오른쪽 차선 피팅
  - 노란색 점: 추정된 차선 중심

**Python vs C++ 차이:**
| 항목 | Python (`camera`) | C++ (`camera_cpp`) |
|------|-------------------|---------------------|
| 속도 | 느림 (Python 오버헤드) | 빠름 |
| 수정 반영 | 즉시 (symlink) | 빌드 필요 |
| cv_bridge | 미사용 (순수 numpy) | 사용 |
| 알고리즘 | RANSAC + 2차 다항식 | RANSAC + 2차 다항식 |

### 카메라 단독 테스트

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
# -> /camera/front/image 선택
```

### Arduino 모터 테스트

> **중요:** Arduino bridge는 항상 Python (pyserial) 노드를 사용합니다.
> C++ boost::asio 방식은 시리얼 쓰기 시 silent failure가 발생하므로 사용하지 않습니다.

**터미널 1 - Arduino 노드:**
```bash
ros2 run arduino_driver arduino_bridge_node.py --ros-args \
  --params-file /root/ros2_ws/install/arduino_driver/share/arduino_driver/config/arduino.yaml
```

**터미널 2 - 수동 명령:**
```bash
# 전진 (속도 0.5 m/s)
ros2 topic pub --once /arduino/cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.5, steering_angle: 0.0}}"

# 좌회전
ros2 topic pub --once /arduino/cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.3, steering_angle: 0.26}}"

# 정지
ros2 topic pub --once /arduino/cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.0}}"
```

**긴급 정지 (시리얼 직접):**
```bash
echo "S" > /dev/ttyACM0
```

### Arduino 명령 모드

현재 설정: **연속 모드** (V:pwm,S:servo)

명령 형식: `V:<pwm값>,S:<서보값>`

| 예시 | 설명 |
|------|------|
| `V:150,S:90` | PWM 150, 서보 중앙(90도) |
| `V:200,S:70` | PWM 200, 좌회전 |
| `V:0,S:90` | 정지 |

**연속 모드 장점:**
- 더 정밀한 속도 제어 (0~255 PWM)
- 더 정밀한 조향 제어 (서보 각도 직접)
- 불필요한 명령 변환 없음

---

## YOLOv8n-seg 실행

YOLOv8n-seg 노드는 카메라 영상에서 신호등, 장애물, 주행 가능 영역을 감지합니다.

### 실행 방법

```bash
# start.sh로 실행
./start.sh yolo

# 또는 컨테이너 내부에서 직접 실행
ros2 run perception_pkg yolov8n_seg_node.py
```

### 기능

1. 신호등 감지 + HSV 색상 분석 -> 빨간불 정지, 초록불 주행
2. 장애물 감지 (보행자, 차량)
3. 주행 가능 영역 마스크 -> OpenCV 차선 인식 보조
4. 디버그 오버레이 -> rviz2 Image 패널

### 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/traffic_light_state` | String | 신호등 상태 (red/green/yellow/unknown) |
| `/perception/traffic_light_detected` | Bool | 신호등 감지 여부 |
| `/yolo/overlay` | Image | YOLO 디버그 오버레이 (시각화용) |
| `/perception/drivable_mask` | Image | 주행 가능 영역 마스크 |
| `/perception/yolo_obstacles` | Float32MultiArray | 장애물 bbox [x1,y1,x2,y2,conf,...] |

### rviz2에서 확인

1. `rviz2` 실행
2. Add > By topic > `/yolo/overlay` > Image 추가
3. YOLO 감지 결과가 오버레이된 영상을 실시간으로 확인 가능

---

## 전체 시스템 실행

### Track 모드 (차선 추종)

```bash
# start.sh로 간편 실행
./start.sh run          # track_launch.py 실행
./start.sh test         # test_mode로 실행

# 또는 직접 실행
ros2 launch bringup track_launch.py
```

**실행되는 노드:**
- `usb_cam_node_exe` - 카메라
- `lane_tracking_node` - 차선 추적
- `lane_marking_node` - 차선 표시 검출
- `arduino_bridge_node` - 모터 제어 (Python/pyserial)
- `ultrasonic_processor_node` - 초음파 센서
- `lidar_obstacle_node` - 라이다 장애물
- `decision_node` - 주행 결정

### 옵션

```bash
# AI 결정 모드
ros2 launch bringup track_launch.py decision_mode:=ai

# 테스트 모드 (센서 없이 모터 구동)
ros2 launch bringup track_launch.py test_mode:=true
```

> **참고:** `use_cpp:=false`로 Python 차선 인식 노드를 사용할 수 있습니다 (ROS2 변환 완료, cv_bridge 미사용).
> Arduino bridge는 항상 Python을 사용합니다 (C++ boost::asio 시리얼 문제).

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

### 주요 토픽추론 혹시 쿠다로 되어있어 CPU가 아니라?

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/front/image` | Image | 카메라 원본 영상 |
| `/lane_overlay` | Image | 차선 인식 시각화 |
| `/lane/steering_angle` | Float32 | 차선 기반 조향 각도 |
| `/lane/center_offset` | Float32 | 차선 중심 오프셋 |
| `/arduino/cmd` | AckermannDriveStamped | 최종 모터 명령 |
| `/cmd_vel` | AckermannDriveStamped | 속도/조향 명령 |
| `/scan` | LaserScan | LiDAR 스캔 데이터 |
| `/perception/traffic_light_state` | String | 신호등 상태 |
| `/perception/yolo_obstacles` | Float32MultiArray | YOLO 장애물 좌표 |
| `/yolo/overlay` | Image | YOLO 디버그 오버레이 |
| `/perception/drivable_mask` | Image | 주행 가능 영역 마스크 |

### 토픽 모니터링

```bash
# 조향 각도 실시간 확인
ros2 topic echo /lane/steering_angle

# 모터 명령 확인
ros2 topic echo /arduino/cmd

# 발행 속도 확인
ros2 topic hz /lane/steering_angle

# 신호등 상태 확인
ros2 topic echo /perception/traffic_light_state
```

### 이미지 시각화

```bash
ros2 run rqt_image_view rqt_image_view
```

**선택 가능한 토픽:**
- `/camera/front/image` - 원본 카메라 영상
- `/lane_overlay` - 차선 인식 오버레이
- `/yolo/overlay` - YOLO 감지 결과 오버레이

### rviz2 시각화

```bash
rviz2
```

**설정 방법:**
1. Global Options > Fixed Frame: `base_link`으로 설정 (map 아님)
2. Add > By topic 에서 다음을 추가:
   - `/yolo/overlay` > Image -- YOLO 감지 결과
   - `/camera/front/image` > Image -- 원본 카메라 영상
   - `/lane_overlay` > Image -- 차선 인식 결과
   - `/scan` > LaserScan -- LiDAR 스캔 데이터

> **주의:** Image를 추가할 때 `Camera`가 아닌 `Image` display를 선택해야 합니다. Camera display는 camera_info 토픽이 필요합니다.

### 전체 시스템 모니터링

```bash
rqt
# -> Plugins > Topics > Topic Monitor
# -> Plugins > Visualization > Image View
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
# start.sh 사용 (권장)
./start.sh restart

# 수동 방법
cd /home/deepblue/target_projects/adas_env
docker compose down
docker compose up -d
```

### 차선 인식이 안 될 때

1. 카메라 영상 확인: `./start.sh camera`로 원본 영상 확인
2. 조명 조건 확인 (너무 밝거나 어두우면 안 됨)
3. 흰색/노란색 선이 카메라에 보이는지 확인
4. 파라미터 조정 (`lane_params.yaml`):
   ```yaml
   canny_low: 25      # 낮추면 더 민감 (현재 25)
   canny_high: 80     # 낮추면 더 민감 (현재 80)
   roi_y_ratio: 0.55  # ROI 시작 비율 (화면 하단 45%)
   kp: 0.7            # 조향 비례 게인
   avg_window: 2      # 이동평균 크기 (낮으면 반응 빠름, 높으면 안정)
   ```

### 차선 피팅(파란/빨간 선)이 어긋날 때

현재 RANSAC + 2차 다항식 피팅을 사용합니다. 튜닝 가능한 값:

- `detector.py` 또는 `lane_geometry.cpp`의 RANSAC 파라미터:
  - `ransac_iters`: 반복 횟수 (기본 60, 늘리면 정확도↑ 속도↓)
  - `inlier_thresh`: 인라이어 판정 거리 (기본 12px, 줄이면 엄격)
  - `degree`: 다항식 차수 (기본 2, 직선 구간에서는 1도 가능)
- HoughLinesP 파라미터: `threshold=25, minLineLength=10, maxLineGap=150`
- slope 임계값: `0.12` (약 7도 미만 수평선 제거)

### 모터가 안 돌아갈 때

1. Arduino 연결 확인: `ls /dev/ttyACM0`
2. arduino_bridge 노드 실행 확인: `ros2 node list | grep arduino`
3. 명령 발행 확인: `ros2 topic echo /arduino/cmd`
4. test_mode 사용: `ros2 launch bringup track_launch.py test_mode:=true`
5. 시리얼 직접 테스트:
   ```bash
   python3 -c "
   import serial, time
   s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
   time.sleep(2)
   s.write(b'V:128,S:90\n')
   time.sleep(0.1)
   print(s.readline())
   "
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

- [ ] Docker 이미지 빌드 (`./start.sh build` 또는 `docker compose build`)
- [ ] Docker 컨테이너 실행 (`./start.sh` 또는 `docker compose up -d`)
- [ ] X11 권한 설정 (start.sh 사용 시 자동)
- [ ] ROS2 워크스페이스 빌드 (`./start.sh ros-build` 또는 `colcon build --symlink-install`)
- [ ] 카메라 + 차선 인식 테스트 Python (`./start.sh camera`)
- [ ] 카메라 + 차선 인식 테스트 C++ (`./start.sh camera_cpp`)
- [ ] 차선 인식 확인 (`/lane_overlay` 파란/빨간 곡선이 차선 따르는지)
- [ ] 모터 수동 테스트 (`ros2 topic pub /arduino/cmd ...`)
- [ ] YOLO 노드 테스트 (`./start.sh yolo`)
- [ ] 전체 시스템 테스트 (`./start.sh run`)

---

## 컨테이너 재접속 vs 재시작

| 상황 | 명령어 | 설치한 패키지 | ROS2 환경 |
|------|--------|--------------|-----------|
| 터미널 닫고 다시 접속 | `docker exec -it adas_container bash` | 유지됨 | 자동 설정 |
| 컨테이너 재시작 | `./start.sh restart` | **Dockerfile에 포함되어 유지** | 자동 설정 |
| 이미지 재빌드 | `./start.sh build` | 새로 설치 | 자동 설정 |