# Code Change Log

코드 변경 이력을 기록합니다.

---

## 2026-01-29

### 1. README.md 전체 재작성

**커밋:** `f210a94`

**변경 내용:**
- 프로젝트 구조 문서화 (8개 ROS2 패키지)
- 시스템 아키텍처 다이어그램 추가
- 하드웨어 연결 가이드 (Arduino, LiDAR, Camera)
- 설정 파일 예시 (yaml)
- 빌드 및 실행 방법
- 토픽 목록 (센서, 인식, 제어)
- Arduino 명령 프로토콜 문서화
- 실차 테스트 절차

---

### 2. requirements.txt 추가

**커밋:** `3ba7db7`

**파일:** `requirements.txt`

**내용:**
```
numpy>=1.21.0
opencv-python>=4.5.0
ultralytics>=8.0.0
pyserial>=3.5
```

---

### 3. README.md 의존성 설치 섹션 추가

**커밋:** `3ba7db7`

**추가된 내용:**
- ROS2 apt 패키지 설치 명령
- Python pip 패키지 설치 명령

---

### 4. .gitignore 업데이트

**커밋:** `f210a94`

**추가된 항목:**
```
.vscode/
*.db
*.ipch
build/
install/
log/
```

---

### 5. Docker 환경 설정

**파일:** `/home/deepblue/target_projects/adas_env/compose.yaml`

**변경 내용:**
- YAML 들여쓰기 수정
- 볼륨 마운트 경로 수정: `ros2_autonomous_cpp` → `/root/ros2_ws`
- 디바이스 추가: `/dev/ttyACM0`, `/dev/ttyUSB0`, `/dev/video0`
- `privileged: true` 추가

**최종 설정:**
```yaml
services:
  adas-dev:
    image: osrf/ros:humble-desktop
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
```

---

### 6. Config 파일 TODO 제거 및 실제 값 설정

**파일들:**
- `src/drivers/usb_cam_driver/config/usb_cam.yaml`
- `src/drivers/rplidar_driver/config/rplidar.yaml`
- `src/drivers/ultrasonic_driver/config/ultrasonic.yaml`

**변경 내용:**
- TODO 주석 제거
- 실제 연결된 장치 경로 설정:
  - Camera: `/dev/video0`
  - LiDAR: `/dev/ttyUSB0`
  - Arduino: `/dev/ttyACM0`

**usb_cam.yaml:**
```yaml
video_device: /dev/video0
image_width: 640
image_height: 480
pixel_format: yuyv
framerate: 30
frame_id: camera_front
camera_topic: /camera/front/image
```

**rplidar.yaml:**
```yaml
serial_port: /dev/ttyUSB0
serial_baudrate: 115200
frame_id: laser
inverted: false
angle_compensate: true
min_range: 0.15
max_range: 8.0
```

**ultrasonic.yaml:**
```yaml
safe_distance_m: 0.2
ranges_topic: /ultrasonic/ranges
```

---

### 7. USB 웹캠 설정 변경 (Logitech C920)

**파일:** `src/drivers/usb_cam_driver/config/usb_cam.yaml`

**변경 내용:**
```yaml
# 변경 전
video_device: "/dev/video0"

# 변경 후
video_device: "/dev/video2"  # 이후 /dev/video4로 수정
```

---

### 8. usb_cam_launch.py 기본값 변경

**파일:** `src/drivers/usb_cam_driver/launch/usb_cam_launch.py`

**변경 내용:**
```python
# 변경 전
video_device_arg = DeclareLaunchArgument(
    'video_device',
    default_value='/dev/video0',
    ...
)

# 변경 후
video_device_arg = DeclareLaunchArgument(
    'video_device',
    default_value='/dev/video4',
    ...
)
```

---

### 9. compose.yaml USB 웹캠 장치 추가

**파일:** `/home/deepblue/target_projects/adas_env/compose.yaml`

**변경 내용:**
```yaml
devices:
  - /dev/ttyACM0:/dev/ttyACM0
  - /dev/ttyUSB0:/dev/ttyUSB0
  - /dev/video0:/dev/video0
  - /dev/video4:/dev/video4  # USB 웹캠 추가
```

---

### 10. how_to_run.md 문서 추가

**파일:** `docs/how_to_run.md`

**내용:**
- Docker 환경 설정 방법
- 컨테이너 실행 및 접속 방법
- ROS2 빌드 방법
- 개별 테스트 (카메라, Arduino)
- 전체 시스템 실행 방법
- 모니터링 및 시각화
- 문제 해결 가이드
- 멀티 터미널 사용법
- 테스트 체크리스트

---

### 11. Arduino 명령 모드 변경 (Continuous 모드)

**파일:** `src/drivers/arduino_driver/config/arduino.yaml`

**변경 내용:**
```yaml
# 변경 전
use_legacy_cmd: true  # F/B/L/R/l/r/S 명령

# 변경 후
use_legacy_cmd: false  # V:pwm,S:servo 연속 모드
```

**설명:**
- 기존 레거시 모드: `F` (전진), `B` (후진), `L/l` (좌회전), `R/r` (우회전), `S` (정지)
- 연속 모드: `V:pwm` (PWM 값 직접 전달), `S:servo` (서보 각도 직접 전달)
- 연속 모드가 더 정밀한 제어 가능

---

### 12. Decision Node 테스트 모드 추가

**파일들:**
- `src/decision/include/decision/decision_node.hpp`
- `src/decision/src/decision_node.cpp`
- `src/bringup/launch/track_launch.py`

**변경 내용:**
- `test_mode` 파라미터 추가 (기본값: false)
- 테스트 모드 활성화 시 센서 체크 우회
  - LiDAR 장애물 체크 우회
  - 초음파 안전 거리 체크 우회
  - 신호등 체크 우회
  - 차선 데이터 타임아웃 체크 우회

**decision_node.hpp:**
```cpp
bool test_mode_;  // 테스트 모드: 센서 없이 모터 구동 가능
```

**decision_node.cpp:**
```cpp
// 파라미터 선언
this->declare_parameter("test_mode", false);
test_mode_ = this->get_parameter("test_mode").as_bool();

// timerCallback에서 테스트 모드 체크
if (test_mode_) {
    cmd.speed = cruise_speed_;
    cmd.steering_angle = mapSteer(lane_steer_norm_);
    cmd_pub_->publish(cmd);
    return;  // 센서 체크 우회
}
```

**track_launch.py:**
```python
# Launch argument 추가
test_mode_arg = DeclareLaunchArgument(
    'test_mode',
    default_value='false',
    description='Test mode: bypass sensor checks for motor testing'
)

# 사용 예시
ros2 launch bringup track_launch.py test_mode:=true
```

---

### 13. 차선 인식 개선 (코너링, 점선, 딜레이)

**문제점:**
1. **코너링 (곡선 도로)**: 곡선에서 차선 감지 실패
2. **점선 차선**: 짧은 점선 세그먼트 놓침
3. **딜레이**: 이동평균 필터로 인한 응답 지연

**수정 파일:**
- `src/perception_pkg/src/lane_tracking_node.cpp`
- `src/perception_pkg/perception_pkg/perception/lane/detector.py`
- `src/perception_pkg/config/lane_params.yaml`

---

#### 변경 1: HoughLinesP 파라미터 (곡선/점선 개선)

**파일:** `lane_tracking_node.cpp:129`, `detector.py:36`

| 파라미터 | 변경 전 | 변경 후 | 이유 |
|----------|---------|---------|------|
| `threshold` | 50 | 30 | 곡선에서 직선 투표수 부족 → 낮춰서 감지율 향상 |
| `minLineLength` | 40 | 20 | 짧은 점선 세그먼트(20~30px) 감지 불가 → 낮춰서 감지 |
| `maxLineGap` | 50 | 100 | 점선 간격(50px 이상) 연결 안됨 → 늘려서 연결 |

**C++ 코드 비교:**
```cpp
// 변경 전
cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 40, 50);

// 변경 후
cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 20, 100);
```

**Python 코드 비교:**
```python
# 변경 전
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                        minLineLength=40, maxLineGap=50)

# 변경 후
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30,
                        minLineLength=20, maxLineGap=100)
```

---

#### 변경 2: slope 필터 완화 (급커브 개선)

**파일:** `lane_tracking_node.cpp:146`, `detector.py:48`

| 파라미터 | 변경 전 | 변경 후 | 이유 |
|----------|---------|---------|------|
| `slope 임계값` | 0.3 | 0.15 | 급커브에서 거의 수평인 선도 필요 → 완화 |

**코드 비교:**
```cpp
// 변경 전: 기울기 0.3 미만 제거 (약 17도)
if (std::abs(slope) < 0.3) continue;

// 변경 후: 기울기 0.15 미만 제거 (약 8.5도)
if (std::abs(slope) < 0.15) continue;
```

**효과:**
- 변경 전: 17도 이하 선 제거 → 급커브에서 차선 놓침
- 변경 후: 8.5도 이하만 제거 → 급커브에서도 차선 감지

---

#### 변경 3: 파라미터 튜닝 (딜레이 감소)

**파일:** `lane_params.yaml`

| 파라미터 | 변경 전 | 변경 후 | 이유 |
|----------|---------|---------|------|
| `canny_low` | 50 | 30 | 더 민감한 엣지 검출 |
| `canny_high` | 150 | 100 | 더 민감한 엣지 검출 |
| `avg_window` | 5 | 3 | 딜레이 감소 (166ms → 100ms @30fps) |

**코드 비교:**
```yaml
# 변경 전
canny_low: 50
canny_high: 150
avg_window: 5

# 변경 후
canny_low: 30
canny_high: 100
avg_window: 3
```

**딜레이 계산:**
- 변경 전: 5프레임 × 33ms = **166ms** 지연
- 변경 후: 3프레임 × 33ms = **100ms** 지연
- **개선: 40% 딜레이 감소**

---

#### 예상 효과

| 문제 | 개선 전 | 개선 후 |
|------|---------|---------|
| 코너링 | 곡선에서 차선 놓침 | threshold/slope 완화로 감지율 향상 |
| 점선 | 짧은 점선 놓침 | minLineLength 축소로 감지 |
| 딜레이 | 166ms | 100ms (40% 개선) |

---

### 14. 2026 경기도 대회 트랙 최적화

**날짜:** 2026-01-30

**배경:**
- 대회 트랙 사양: 도로폭 850mm, 차선폭 50mm, 우측 차선 반시계 주행
- 문제: 외측 실선은 잘 인식되나 중앙 점선 인식 어려움

**수정 파일:**
- `src/perception_pkg/src/lane_tracking_node.cpp`
- `src/perception_pkg/perception_pkg/perception/lane/detector.py`
- `src/perception_pkg/config/lane_params.yaml`

---

#### 변경 1: HoughLinesP 파라미터 (점선 감지 개선)

| 파라미터 | 변경 전 | 변경 후 | 이유 |
|----------|---------|---------|------|
| `threshold` | 30 | 25 | 점선 차선 투표수 낮음 대응 |
| `minLineLength` | 20 | 10 | 점선 세그먼트 (10~15px) 감지 |
| `maxLineGap` | 100 | 150 | 점선 간격 더 넓게 연결 |

```cpp
// 변경 전
cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 20, 100);

// 변경 후
cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 25, 10, 150);
```

---

#### 변경 2: slope 임계값 완화 (급커브 대응)

| 파라미터 | 변경 전 | 변경 후 | 이유 |
|----------|---------|---------|------|
| `slope 임계값` | 0.15 | 0.12 | 대회 트랙 급커브 대응 (약 7도까지 허용) |

```cpp
// 변경 전: 약 8.5도 미만 제거
if (std::abs(slope) < 0.15) continue;

// 변경 후: 약 7도 미만 제거
if (std::abs(slope) < 0.12) continue;
```

---

#### 변경 3: lane_params.yaml 튜닝

| 파라미터 | 변경 전 | 변경 후 | 이유 |
|----------|---------|---------|------|
| `kp` | 0.6 | 0.7 | 850mm 트랙 폭에 맞춰 응답성 향상 |
| `canny_low` | 30 | 25 | 점선 엣지 민감도 향상 |
| `canny_high` | 100 | 80 | 점선 대비 낮음 대응 |
| `avg_window` | 3 | 2 | 곡선 응답 개선 (66ms @30fps) |

```yaml
# 변경 전
kp: 0.6
canny_low: 30
canny_high: 100
avg_window: 3

# 변경 후
kp: 0.7
canny_low: 25
canny_high: 80
avg_window: 2
```

---

#### 변경 4: 단일 차선 폴백 로직 추가 (신규)

**문제:** 점선(중앙선)이 감지되지 않으면 차선 중앙 계산 오류

**해결:** 한쪽 차선만 감지 시 차선 폭을 기준으로 중앙 추정

```cpp
// 양쪽 감지: 평균
// 오른쪽(실선)만 감지: 왼쪽으로 차선폭 35% 이동
// 왼쪽(점선)만 감지: 오른쪽으로 차선폭 35% 이동

if (left_detected && right_detected) {
    center_x = mean(lane_positions);
} else if (right_detected && !left_detected) {
    int estimated_lane_width = w * 0.35;  // 850mm/2 ≈ 35%
    center_x = lane_positions[0] - estimated_lane_width / 2;
} else if (left_detected && !right_detected) {
    int estimated_lane_width = w * 0.35;
    center_x = lane_positions[0] + estimated_lane_width / 2;
} else {
    center_x = w / 2.0;  // 직진 유지
}
```

---

#### 예상 효과

| 문제 | 개선 전 | 개선 후 |
|------|---------|---------|
| 점선 감지 | 자주 놓침 | minLineLength/maxLineGap 조정으로 감지율 향상 |
| 급커브 | 일부 구간 놓침 | slope 0.12로 완화 |
| 단일 차선 | 이미지 중앙 기본값 | 차선 폭 기준 추정 |
| 딜레이 | 100ms | 66ms (34% 개선) |

---

### 15. 전방/후방 듀얼 카메라 시스템 구축

**날짜:** 2026-01-30

**배경:**
- 대회 규정: 수직 주차 미션에서 주차 후 OUT 라인까지 주행 필요
- 후방 카메라로 주차 종료 라인(흰색 수평선) 감지 후 재출발

**카메라 장치 할당:**
| 카메라 | 장치 | 용도 | Launch 파일 |
|--------|------|------|-------------|
| 전방 | `/dev/video6` | 차선 인식, 신호등, 표지판 | track_launch.py, mission_launch.py |
| 후방 | `/dev/video4` | 주차 라인 감지 | mission_launch.py만 |

**생성 파일:**

1. **후방 카메라 설정**
   - `src/drivers/usb_cam_driver/config/usb_cam_rear.yaml`

2. **듀얼 카메라 launch**
   - `src/drivers/usb_cam_driver/launch/dual_usb_cam_launch.py`

3. **후방 카메라 주차 라인 감지 모듈**
   - `src/perception_pkg/perception_pkg/perception/parking/parking_line_detector.py`
   - `src/perception_pkg/perception_pkg/parking_line_node.py`

**수정 파일:**

1. **usb_cam.yaml**
   ```yaml
   # 변경 전
   video_device: "/dev/video4"  # 후방 카메라였음

   # 변경 후
   video_device: "/dev/video6"  # 전방 카메라로 변경
   ```

2. **usb_cam_launch.py**
   ```python
   # 변경 전
   default_value='/dev/video4'

   # 변경 후
   default_value='/dev/video6'  # 기본값 = 전방
   ```

3. **mission_launch.py**
   ```python
   # 변경 전 (단일 카메라)
   usb_cam_launch = IncludeLaunchDescription(...)

   # 변경 후 (듀얼 카메라 + 주차 라인 노드)
   dual_usb_cam_launch = IncludeLaunchDescription(...)
   parking_line_node = Node(...)
   ```

4. **CMakeLists.txt**
   ```cmake
   # parking_line_node.py 추가
   install(PROGRAMS
     ...
     perception_pkg/parking_line_node.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

---

#### 주차 라인 감지 알고리즘

```python
def detect_parking_end_line(image):
    # 1. ROI: 하단 40% 영역만 사용
    roi = image[roi_y_start:h, 0:w]

    # 2. Canny 엣지 검출
    edges = cv2.Canny(blur, 50, 150)

    # 3. HoughLinesP로 직선 검출
    lines = cv2.HoughLinesP(edges, threshold=50, minLineLength=100)

    # 4. 거의 수평인 선 필터링 (기울기 < 0.3)
    if slope < 0.3:
        horizontal_lines.append(line)

    # 5. 수평선 2개 이상 감지 시 주차 라인으로 판정
    if len(horizontal_lines) >= 2:
        return True
```

**발행 토픽:**
- `/parking/line_detected` (Bool): 주차 라인 감지 여부
- `/parking/overlay` (Image): 디버그 오버레이

---

#### Launch 파일별 카메라 구성

**track_launch.py** (트랙 주행 모드)
- 전방 카메라만 (/dev/video6)
- LiDAR, 초음파, 후방 카메라 미사용

**mission_launch.py** (미션 수행 모드)
- 전방 카메라 (/dev/video6): 차선, 신호등
- 후방 카메라 (/dev/video4): 주차 라인
- LiDAR, 초음파 전부 사용

---

#### Docker 컨테이너 장치 마운트

```yaml
# compose.yaml
devices:
  - /dev/ttyACM0:/dev/ttyACM0  # Arduino
  - /dev/ttyUSB0:/dev/ttyUSB0  # LiDAR
  - /dev/video4:/dev/video4     # 후방 카메라
  - /dev/video5:/dev/video5
  - /dev/video6:/dev/video6     # 전방 카메라
  - /dev/video7:/dev/video7
```

---

### 16. RPLiDAR 드라이버 런치 파일 추가

**날짜:** 2026-01-30

**배경:**
- RViz2에서 LiDAR 포인트 클라우드가 보이지 않는 문제 발생
- Static TF Publisher (base_link → laser)는 추가했으나 실제 RPLiDAR 드라이버가 실행되지 않음
- TF는 센서의 **위치**만 정의하고, 드라이버는 실제 **센서 데이터**를 발행

**문제 분석:**
- `/scan` 토픽이 발행되지 않음 (드라이버 미실행)
- TF 트리는 정상이나 LaserScan 데이터 없음
- `rplidar_launch.py` 파일은 존재하나 런치 파일에서 호출하지 않음

**수정 파일:**
1. `src/bringup/launch/track_launch.py`
2. `src/bringup/launch/mission_launch.py`

---

#### 변경 내용

**track_launch.py** (라인 271-280)

```python
# 추가됨: RPLiDAR 드라이버 실행
# Include RPLiDAR launch
rplidar_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('rplidar_driver'),
            'launch',
            'rplidar_launch.py'
        ])
    ])
)

return LaunchDescription([
    decision_mode_arg,
    camera_topic_arg,
    use_compressed_arg,
    use_cpp_arg,
    test_mode_arg,
    usb_cam_launch,
    lane_perception_launch,
    rplidar_launch,  # 추가
    OpaqueFunction(function=launch_setup),
])
```

**mission_launch.py** (라인 295-304, 314)

```python
# 변경 전 (주석 처리됨)
# NOTE: Ensure rplidar_ros is running separately if needed
# rplidar_launch = IncludeLaunchDescription(...)

# 변경 후 (주석 해제 및 활성화)
# Include RPLiDAR launch
rplidar_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('rplidar_driver'),
            'launch',
            'rplidar_launch.py'
        ])
    ])
)

return LaunchDescription([
    decision_mode_arg,
    camera_topic_arg,
    use_compressed_arg,
    use_cpp_arg,
    dual_usb_cam_launch,
    parking_line_node,
    lane_perception_launch,
    rplidar_launch,  # 주석 해제
    OpaqueFunction(function=launch_setup),
])
```

---

#### RPLiDAR 드라이버 역할

**rplidar_launch.py가 실행하는 노드:**
- Package: `rplidar_ros`
- Executable: `rplidar_node`
- 기능:
  - `/dev/ttyUSB0`에서 RPLiDAR A1 센서 데이터 읽기
  - `/scan` 토픽으로 LaserScan 메시지 발행
  - 프레임: `laser`

**Static TF Publisher와의 관계:**
```
Static TF Publisher: base_link → laser (위치 정보)
RPLiDAR Driver:      /scan 토픽 발행 (센서 데이터)
                     ↓
RViz2:               LaserScan 표시 (위치 + 데이터 결합)
```

---

#### 패키지 설치

**문제:** 런치 파일 실행 시 에러 발생
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
"package 'rplidar_ros' not found, searching: [...]"
```

**원인:**
- `rplidar_driver`는 wrapper 패키지 (launch, config만 포함)
- `rplidar_ros`는 SLAMTEC 공식 ROS2 드라이버 (실제 실행 파일 포함)
- rplidar_launch.py가 rplidar_ros의 rplidar_node를 실행하므로 설치 필요

**해결:**

**방법 1: 현재 컨테이너에 수동 설치 (임시)**
```bash
# Docker 컨테이너에 rplidar_ros 설치
apt-get update
apt-get install -y ros-humble-rplidar-ros

# 설치 확인
source /opt/ros/humble/setup.bash
ros2 pkg list | grep rplidar
# 출력: rplidar_ros
```

**방법 2: Dockerfile 수정 (영구적)**
```dockerfile
# /home/deepblue/target_projects/adas_env/Dockerfile
# 필수 ROS2 패키지 및 pip 설치
RUN apt update && apt install -y \
    ros-humble-usb-cam \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rplidar-ros \  # 추가
    v4l-utils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
```

**이미지 재빌드:**
```bash
cd /home/deepblue/target_projects/adas_env
docker compose build
docker compose down
docker compose up -d
```

**이점:**
- 컨테이너 재시작 시 자동으로 rplidar_ros 포함
- 수동 설치 불필요

---

#### 검증 방법

```bash
# 1. /scan 토픽 확인
ros2 topic hz /scan
# 출력: average rate: 10.000 (RPLiDAR A1은 10Hz)

# 2. TF 트리 확인
ros2 run tf2_tools view_frames
# frames.pdf 생성 → base_link → laser 확인

# 3. RViz2에서 시각화
rviz2
# Fixed Frame: base_link
# Add → LaserScan
# Topic: /scan
# → 빨간색 포인트 클라우드 표시됨
```

---

## 예정된 변경

- [ ] 차선 추적 YOLO 모델 적용 (GPU 환경 필요)
- [ ] Docker GPU 지원 추가
- [x] 실차 테스트 후 파라미터 튜닝 - 완료 (HoughLinesP, slope, Canny)
- [x] Dockerfile 작성 (필수 패키지 자동 설치) - 완료
- [x] 2026 경기도 대회 트랙 최적화 - 완료 (점선 감지, 단일 차선 폴백)
- [x] 전방/후방 듀얼 카메라 시스템 - 완료 (주차 라인 감지)
- [x] RPLiDAR 드라이버 런치 파일 추가 - 완료
- [ ] 실제 센서 위치 측정 및 TF 업데이트 (sensor_calibration.md 참고)
- [ ] 카메라 캘리브레이션 파일 생성
- [ ] 2차 다항식 피팅으로 곡선 근사 개선 (추가 개선 필요시)
