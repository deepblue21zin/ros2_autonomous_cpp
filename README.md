# ROS2 Autonomous Vehicle

ROS2 Humble 기반 자율주행 시스템입니다. 차선 추적, 장애물 감지, 신호등 인식 등의 기능을 제공합니다.

## 시스템 요구사항

- Ubuntu 22.04
- ROS2 Humble
- OpenCV 4.x
- Boost (Asio)

## 프로젝트 구조

```
src/
├── bringup/                    # 런치 파일 (시스템 통합)
│   └── launch/
│       ├── track_launch.py     # 트랙 모드 (기본 차선 추적)
│       ├── mission_launch.py   # 미션 모드 (전체 기능)
│       ├── obstacle.launch     # 장애물 회피 모드
│       └── parking.launch      # 주차 모드
│
├── common/                     # 공통 유틸리티
│   └── scripts/
│       ├── lidar_monitor.py    # LiDAR 상태 모니터링
│       └── ultrasonic_monitor.py
│
├── drivers/                    # 하드웨어 드라이버
│   ├── arduino_driver/         # Arduino 통신 (차량 제어)
│   ├── ultrasonic_driver/      # 초음파 센서 처리
│   ├── rplidar_driver/         # LiDAR 래퍼
│   └── usb_cam_driver/         # USB 카메라 래퍼
│
├── perception_pkg/             # 인식 모듈
│   ├── src/                    # C++ 노드
│   │   ├── lane_tracking_node.cpp      # 차선 추적
│   │   ├── lane_marking_node.cpp       # 차선 마킹
│   │   └── obstacle_detection_node.cpp # 장애물 감지
│   └── scripts/                # Python 노드
│       ├── traffic_light_node.py       # 신호등 인식 (YOLO)
│       └── speed_sign_node.py          # 속도 표지판 (YOLO)
│
└── decision/                   # 제어 결정
    ├── src/
    │   ├── decision_node.cpp           # 메인 결정 노드
    │   └── lidar_obstacle_node.cpp     # LiDAR 장애물 감지
    └── scripts/
        ├── decision_node_2026.py       # Python 결정 노드
        └── decision_node_ai.py         # AI 모드
```

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              센서 입력                                   │
├─────────────────────────────────────────────────────────────────────────┤
│  USB Camera         RPLiDAR              Arduino (초음파)                │
│  /dev/video0        /dev/ttyUSB0         /dev/ttyACM0                   │
│       │                  │                    │                          │
│       ▼                  ▼                    ▼                          │
│  ┌─────────┐       ┌──────────┐        ┌──────────────┐                 │
│  │usb_cam  │       │rplidar_  │        │arduino_bridge│                 │
│  │_driver  │       │ros       │        │_node         │                 │
│  └────┬────┘       └────┬─────┘        └──────┬───────┘                 │
│       │                  │                    │                          │
│       ▼                  ▼                    ▼                          │
│ /camera/front/     /scan              /ultrasonic/ranges                │
│ image                                                                    │
└─────────────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────────────────┐
│                              인식 처리                                   │
├─────────────────────────────────────────────────────────────────────────┤
│                              │                                           │
│       ┌──────────────────────┼──────────────────────┐                   │
│       ▼                      ▼                      ▼                   │
│ ┌───────────────┐    ┌───────────────┐    ┌─────────────────┐          │
│ │lane_tracking  │    │obstacle_      │    │ultrasonic_      │          │
│ │_node          │    │detection_node │    │processor_node   │          │
│ └───────┬───────┘    └───────┬───────┘    └────────┬────────┘          │
│         │                    │                     │                    │
│         ▼                    ▼                     ▼                    │
│ /lane/steering_angle  /perception/         /ultrasonic/min_range       │
│                       obstacle_flag                                     │
│                       obstacle_bias                                     │
│                                                                         │
│ ┌───────────────┐    ┌───────────────┐                                 │
│ │lidar_obstacle │    │traffic_light  │                                 │
│ │_node          │    │_node          │                                 │
│ └───────┬───────┘    └───────┬───────┘                                 │
│         │                    │                                          │
│         ▼                    ▼                                          │
│ /perception/          /perception/                                      │
│ obstacle_flag         traffic_light_state                               │
└─────────────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────────────────┐
│                              제어 결정                                   │
├─────────────────────────────────────────────────────────────────────────┤
│                              │                                           │
│                              ▼                                           │
│                     ┌───────────────┐                                   │
│                     │ decision_node │  (10Hz)                           │
│                     │               │                                   │
│                     │ 입력:         │                                   │
│                     │ - 차선 조향각 │                                   │
│                     │ - 장애물 감지 │                                   │
│                     │ - 초음파 거리 │                                   │
│                     │ - 신호등 상태 │                                   │
│                     └───────┬───────┘                                   │
│                             │                                            │
│                             ▼                                            │
│                     /decision/cmd (→ /arduino/cmd)                      │
│                     (AckermannDrive 메시지)                              │
└─────────────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────────────────┐
│                              액추에이터                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                              │                                           │
│                              ▼                                           │
│                     ┌───────────────┐                                   │
│                     │arduino_bridge │                                   │
│                     │_node          │                                   │
│                     └───────┬───────┘                                   │
│                             │                                            │
│                             ▼                                            │
│                     시리얼 통신 (/dev/ttyACM0)                          │
│                     - 레거시 모드: F/B/L/R/l/r/S                        │
│                     - 연속 모드: V:pwm,S:servo                          │
└─────────────────────────────────────────────────────────────────────────┘
```

## 하드웨어 연결

| 장치 | 포트 | 설정 |
|------|------|------|
| Arduino | `/dev/ttyACM0` | 115200 baud |
| RPLiDAR | `/dev/ttyUSB0` | 115200 baud |
| USB Camera | `/dev/video4` | 640x480, 30fps (Logitech C920) |

> **참고:** 노트북 내장 카메라가 `/dev/video0`~`/dev/video3`을 사용하므로 USB 웹캠은 `/dev/video4` 사용

## 설정 파일

### Arduino (`src/drivers/arduino_driver/config/arduino.yaml`)
```yaml
port: /dev/ttyACM0
baudrate: 115200
use_legacy_cmd: false         # 연속 모드 (V:pwm,S:servo) - 더 정밀한 제어
max_speed_mps: 2.0            # 최대 속도 (m/s)
max_steer_deg: 30.0           # 최대 조향각 (도)
center_servo_deg: 90.0        # 서보 중앙값 (도)
```

### LiDAR (`src/drivers/rplidar_driver/config/rplidar.yaml`)
```yaml
serial_port: /dev/ttyUSB0
serial_baudrate: 115200
frame_id: laser
inverted: false               # 장착 방향 반전
min_range: 0.15
max_range: 8.0
```

### Camera (`src/drivers/usb_cam_driver/config/usb_cam.yaml`)
```yaml
video_device: /dev/video4     # USB 웹캠 (노트북 내장은 video0~3)
image_width: 640
image_height: 480
pixel_format: yuyv
framerate: 30
camera_topic: /camera/front/image
```

### 차선 추적 (`src/perception_pkg/config/lane_params.yaml`)
```yaml
kp: 0.6                       # 비례 게인
roi_y_ratio: 0.55             # ROI 시작 위치
canny_low: 50
canny_high: 150
```

## Docker 환경 (권장)

Docker 환경을 사용하면 의존성 설치가 자동으로 처리됩니다.

```bash
# 1. X11 권한 설정 (호스트에서)
xhost +local:docker

# 2. 이미지 빌드 (처음 한 번만)
cd /home/deepblue/target_projects/adas_env
docker compose build

# 3. 컨테이너 시작
docker compose up -d

# 4. 컨테이너 접속
docker exec -it adas_container bash

# 5. ROS2 빌드
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

자세한 내용은 [docs/how_to_run.md](docs/how_to_run.md)를 참조하세요.

---

## 의존성 설치 (Docker 미사용 시)

### ROS2 패키지
```bash
sudo apt update
sudo apt install -y \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-usb-cam \
    ros-humble-rplidar-ros \
    libboost-all-dev \
    libopencv-dev \
    libeigen3-dev
```

### Python 패키지
```bash
pip install -r requirements.txt
```

또는 개별 설치:
```bash
pip install numpy opencv-python ultralytics pyserial
```

## 빌드

```bash
cd ~/ros2_autonomous_cpp

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 빌드
colcon build --symlink-install

# 빌드 결과 소스
source install/setup.bash
```

## 실행

### 트랙 모드 (기본 차선 추적)
```bash
ros2 launch bringup track_launch.py
```

### 트랙 모드 옵션
```bash
# Python 노드 사용
ros2 launch bringup track_launch.py use_cpp:=false

# AI 결정 모드
ros2 launch bringup track_launch.py decision_mode:=ai

# 압축 이미지 사용
ros2 launch bringup track_launch.py use_compressed:=true

# 테스트 모드 (센서 없이 모터 구동)
ros2 launch bringup track_launch.py test_mode:=true
```

### 테스트 모드
`test_mode:=true` 옵션을 사용하면 센서 데이터 없이도 모터가 구동됩니다.

**우회되는 체크:**
- LiDAR 장애물 감지
- 초음파 안전 거리
- 신호등 상태
- 차선 데이터 타임아웃

> ⚠️ **주의:** 테스트 모드에서는 안전 기능이 비활성화됩니다. 실제 주행 전 반드시 비활성화하세요.

### 미션 모드 (전체 기능)
```bash
ros2 launch bringup mission_launch.py
```

## 주요 토픽

### 센서 데이터
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/front/image` | `sensor_msgs/Image` | 전방 카메라 이미지 |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 스캔 데이터 |
| `/ultrasonic/ranges` | `std_msgs/Float32MultiArray` | 초음파 센서 (6채널) |

### 인식 결과
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/lane/steering_angle` | `std_msgs/Float32` | 차선 추적 조향각 (-1.0 ~ 1.0) |
| `/lane/center_offset` | `std_msgs/Float32` | 차선 중심 오프셋 |
| `/perception/obstacle_flag` | `std_msgs/Bool` | 장애물 감지 여부 |
| `/perception/obstacle_bias` | `std_msgs/Float32` | 장애물 회피 바이어스 |
| `/perception/traffic_light_state` | `std_msgs/String` | 신호등 상태 (red/yellow/green) |
| `/ultrasonic/min_range` | `std_msgs/Float32` | 최소 초음파 거리 (m) |

### 제어 명령
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/arduino/cmd` | `ackermann_msgs/AckermannDrive` | 차량 제어 명령 |

## 디버깅

### 토픽 모니터링
```bash
# 제어 명령 확인
ros2 topic echo /arduino/cmd

# 차선 추적 확인
ros2 topic echo /lane/steering_angle

# 초음파 데이터 확인
ros2 topic echo /ultrasonic/ranges

# 전체 토픽 목록
ros2 topic list
```

### 노드 상태 확인
```bash
ros2 node list
ros2 node info /decision_node
```

### 센서 모니터링
```bash
ros2 launch common sensor_debug.launch
```

## 제어 로직 (Decision Node)

Decision Node는 10Hz로 동작하며 다음 순서로 제어를 결정합니다:

1. **정지 조건 확인**
   - 장애물 감지 시 정지
   - 초음파 거리 < 안전거리 시 정지
   - 빨간 신호등 시 정지
   - 차선 데이터 타임아웃 시 정지

2. **조향각 계산**
   ```
   steer = lane_steering_angle + obstacle_bias * weight
   steer = clamp(steer, -1.0, 1.0)
   ```

3. **명령 발행**
   - AckermannDrive 메시지로 속도와 조향각 전송

## Arduino 명령 프로토콜

### 레거시 모드 (`use_legacy_cmd: true`)
| 명령 | 동작 |
|------|------|
| `F` | 전진 |
| `B` | 후진 |
| `L` | 좌회전 (>15도) |
| `R` | 우회전 (>15도) |
| `l` | 소프트 좌회전 (5~15도) |
| `r` | 소프트 우회전 (5~15도) |
| `S` | 정지 |

### 연속 모드 (`use_legacy_cmd: false`)
```
V:pwm,S:servo
```
- `pwm`: 0~255 (속도)
- `servo`: 서보 각도 (중앙 기준)

### 초음파 데이터 수신 포맷
```
F:25,FL:30,FR:28,R:0,RL:0,RR:0
```
- 단위: cm (노드에서 m로 변환)
- 센서 순서: F(전방), FL(전방좌), FR(전방우), R(후방), RL(후방좌), RR(후방우)

## 실차 테스트 절차

### 1. 장치 연결 확인
```bash
ls -la /dev/ttyACM*   # Arduino
ls -la /dev/ttyUSB*   # LiDAR
ls -la /dev/video*    # Camera
```

### 2. 권한 설정
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG video $USER
# 로그아웃 후 재로그인
```

### 3. 개별 노드 테스트

**Arduino 통신 테스트:**
```bash
# 터미널 1
ros2 run arduino_driver arduino_bridge_node

# 터미널 2: 정지 명령
ros2 topic pub /arduino/cmd ackermann_msgs/msg/AckermannDrive "{speed: 0.0, steering_angle: 0.0}"
```

**카메라 테스트:**
```bash
ros2 run usb_cam_driver usb_cam_node
ros2 topic hz /camera/front/image
```

### 4. 전체 시스템 실행
```bash
ros2 launch bringup track_launch.py
```

## 주의사항

1. **처음 테스트 시 차량을 들어올린 상태에서** 바퀴가 땅에 닿지 않게 테스트
2. **속도를 낮게 설정** (`max_speed_mps: 0.5`)
3. **비상 정지 버튼** 준비
4. 넓은 공간에서 테스트

## 라이선스

MIT License
