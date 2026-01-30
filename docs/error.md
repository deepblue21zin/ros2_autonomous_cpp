# Error Log

빌드 및 실행 중 발생한 에러와 해결 방법을 기록합니다.

---

## 2026-01-29

### Error 1: CMake Cache Conflict

**에러 메시지:**
```
CMake Error: The current CMakeCache.txt directory /root/ros2_ws/build/arduino_driver/CMakeCache.txt
is different than the directory /home/deepblue/ros2_autonomous_cpp/build/arduino_driver where
CMakeCache.txt was created.
```

**원인:**
- 호스트에서 빌드한 캐시가 Docker 컨테이너 내부 경로와 충돌
- `build/`, `install/`, `log/` 폴더가 다른 환경에서 생성됨

**해결:**
```bash
# 컨테이너 내부에서 빌드 캐시 삭제
docker exec adas_container bash -c "cd /root/ros2_ws && rm -rf build/ install/ log/"

# 다시 빌드
docker exec adas_container bash -c "source /opt/ros/humble/setup.bash && cd /root/ros2_ws && colcon build --symlink-install"
```

---

### Error 2: compose.yaml Indentation Error

**에러 메시지:**
```
services must be a mapping
```

**원인:**
- YAML 파일의 `services:` 키가 들여쓰기되어 있었음

**해결:**
- `services:`를 파일 맨 앞(들여쓰기 없이)으로 수정

---

### Error 3: GitHub Large File Error (820MB)

**에러 메시지:**
```
remote: error: File .vscode/browse.vc.db is 819.95 MB;
this exceeds GitHub's file size limit of 100.00 MB
```

**원인:**
- VSCode의 IntelliSense 캐시 파일이 커밋에 포함됨

**해결:**
```bash
# 커밋 취소
git reset --soft HEAD~1

# .vscode 폴더 스테이징 해제
git restore --staged .vscode/

# .gitignore에 추가 (이미 있음)
# .vscode/
# *.db

# 다시 커밋 & 푸시
git commit -m "message"
git push origin main
```

---

### Error 4: usb_cam Package Not Found

**에러 메시지:**
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
"package 'usb_cam' not found, searching: ['/root/ros2_ws/install/...']"
```

**원인:**
- `ros-humble-usb-cam` 패키지가 Docker 컨테이너에 설치되지 않음

**해결:**
```bash
docker exec adas_container bash -c "apt install -y ros-humble-usb-cam"
```

---

### Error 5: ros2 command not found

**에러 메시지:**
```
bash: ros2: command not found
```

**원인:**
- ROS2 환경 변수가 설정되지 않음

**해결:**
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

---

### Error 6: ROS2 YAML Parameter Format Error

**에러 메시지:**
```
Error: Cannot have a value before ros__parameters at line 3
```

**원인:**
- ROS2 파라미터 YAML 파일은 `ros__parameters:` 섹션이 필요함
- ROS1 스타일 YAML 형식 사용

**해결:**
ROS2 형식으로 변환:

```yaml
# 잘못된 형식 (ROS1)
param1: value1
param2: value2

# 올바른 형식 (ROS2)
/**:
  ros__parameters:
    param1: value1
    param2: value2
```

---

### Error 7: Parameter Type Mismatch

**에러 메시지:**
```
parameter 'framerate' has invalid type: Wrong parameter type, parameter {framerate} is of type {double}, setting it to {integer} is not allowed.
```

**원인:**
- `framerate`가 `double` 타입이어야 하는데 `integer`로 설정됨

**해결:**
```yaml
# 잘못됨
framerate: 30

# 올바름
framerate: 30.0
```

---

### Error 8: String Array Parameter Error

**에러 메시지:**
```
parameter 'allowed_stop_states' has invalid type: Wrong parameter type, parameter {allowed_stop_states} is of type {string}, setting it to {string_array} is not allowed.
```

**원인:**
- C++ 노드는 `"red,yellow"` 형태의 string을 기대
- launch 파일에서 `"['red']"` 형태로 전달

**해결:**
```python
# 잘못됨
'allowed_stop_states': "['red']"

# 올바름
'allowed_stop_states': "red"
```

---

## Warning (무시 가능)

### perception_pkg 빌드 경고

```cpp
warning: unused parameter 'ys' [-Wunused-parameter]
warning: unused variable 'w' [-Wunused-variable]
warning: unused parameter 'roi_y' [-Wunused-parameter]
```

**파일:** `src/perception_pkg/src/lane_marking_node.cpp`

**상태:** 기능에 영향 없음, 추후 정리 예정

---

### Error 9: ackermann_msgs Library Not Found

**에러 메시지:**
```
error while loading shared libraries: libackermann_msgs__rosidl_typesupport_cpp.so:
cannot open shared object file: No such file or directory
```

**원인:**
- `ros-humble-ackermann-msgs` 패키지가 설치되지 않음
- arduino_driver와 decision_node에서 사용

**해결:**
```bash
apt update && apt install -y ros-humble-ackermann-msgs
```

---

### Error 10: Camera Device Wrong (노트북 내장 카메라 실행됨)

**증상:**
- USB 웹캠 대신 노트북 내장 카메라가 켜짐

**원인:**
- `/dev/video0` ~ `/dev/video3` = 노트북 내장 카메라
- `/dev/video4` ~ `/dev/video5` = USB 웹캠 (Logitech C920)

**확인 방법:**
```bash
v4l2-ctl --list-devices
```

**해결:**

1. `usb_cam.yaml` 수정:
```yaml
video_device: "/dev/video4"
```

2. `usb_cam_launch.py` 수정:
```python
video_device_arg = DeclareLaunchArgument(
    'video_device',
    default_value='/dev/video4',  # USB 웹캠 번호
    description='Video device path'
)
```

3. `compose.yaml`에 디바이스 추가:
```yaml
devices:
  - /dev/video4:/dev/video4
```

4. 컨테이너 재시작:
```bash
docker compose down
docker compose up -d
```

---

### Error 11: Camera Process Crash

**에러 메시지:**
```
terminate called after throwing an instance of 'char*'
[ERROR] [usb_cam_node_exe-1]: process has died [pid XXX, exit code -6]
```

**원인:**
- 다른 프로세스가 카메라를 사용 중
- 또는 다른 터미널에서 카메라 노드가 이미 실행 중

**해결:**
```bash
# 기존 카메라 프로세스 종료
pkill -f usb_cam

# 다시 실행
ros2 launch usb_cam_driver usb_cam_launch.py
```

---

### Error 12: Topic Does Not Appear to be Published Yet

**에러 메시지:**
```
WARNING: topic [/lane/steering] does not appear to be published yet
Could not determine the type for the passed topic
```

**원인:**
- 토픽 이름이 잘못됨
- 또는 해당 노드가 실행되지 않음

**해결:**
- 올바른 토픽 이름 확인:
  - `/lane/steering_angle` (O)
  - `/lane/steering` (X)

```bash
# 전체 토픽 목록 확인
ros2 topic list | grep lane
```

---

## Warning (무시 가능)

### Camera Calibration File Not Found

**메시지:**
```
[ERROR] [camera_calibration_parsers]: Unable to open camera calibration file [/root/.ros/camera_info/front_camera.yaml]
[WARN] [usb_cam]: Camera calibration file /root/.ros/camera_info/front_camera.yaml not found
```

**상태:** 기능에 영향 없음 (캘리브레이션 없이도 동작)

---

### Unknown Camera Controls

**메시지:**
```
unknown control 'white_balance_temperature_auto'
unknown control 'exposure_auto'
unknown control 'focus_auto'
```

**상태:** 해당 카메라가 지원하지 않는 기능, 무시 가능

---

## 2026-01-30

### Error 13: traffic_light_node.py Executable Not Found

**에러 메시지:**
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
executable 'traffic_light_node.py' not found on the libexec directory
'/root/ros2_ws/install/perception_pkg/lib/perception_pkg'
```

**원인:**
- Python 스크립트 파일에 실행 권한(+x)이 없음
- CMakeLists.txt의 `install(PROGRAMS ...)` 명령은 실행 권한이 있는 파일만 설치

**확인 방법:**
```bash
# 실행 권한 확인
ls -la src/perception_pkg/scripts/

# 결과 (문제 있는 파일)
-rw-r--r--  traffic_light_node.py      ← 실행 불가
-rw-r--r--  lane_marking_node.py       ← 실행 불가
-rw-r--r--  obstacle_detection_node.py ← 실행 불가
-rwxr-xr-x  lane_tracking_node.py      ← 실행 가능 (정상)
```

**해결:**
```bash
# 1. 실행 권한 추가
chmod +x src/perception_pkg/scripts/traffic_light_node.py
chmod +x src/perception_pkg/scripts/lane_marking_node.py
chmod +x src/perception_pkg/scripts/obstacle_detection_node.py
chmod +x src/perception_pkg/scripts/speed_sign_node.py

# 2. 재빌드
cd /root/ros2_ws
colcon build --symlink-install --packages-select perception_pkg

# 3. 확인
source install/setup.bash
ros2 pkg executables perception_pkg | grep traffic
# perception_pkg traffic_light_node.py  ← 등록됨
```

**결과:**
```
# 모든 Python 노드 정상 등록
✅ traffic_light_node.py
✅ lane_marking_node.py
✅ obstacle_detection_node.py
✅ parking_line_node.py
✅ speed_sign_node.py
```

**원리:**
- CMake의 `install(PROGRAMS ...)` 명령은 Unix 실행 권한이 있는 파일을 `lib/${PROJECT_NAME}/` 디렉토리로 복사
- 실행 권한이 없으면 일반 파일로 취급되어 ROS2가 실행 파일로 인식하지 못함

---

### Error 14: Camera Device Not Mounted in Docker

**증상:**
```
Cannot open device: `/dev/video4`, double-check read / write permissions
[ERROR] [usb_cam]: Device specified is not available or is not a valid V4L2 device
Available V4L2 devices are: /dev/video0 ~ /dev/video3
```

**원인:**
- 호스트에는 `/dev/video4`(USB 웹캠)가 존재하지만 Docker 컨테이너에 마운트되지 않음
- `compose.yaml`의 `devices:` 섹션에 장치가 없거나, 컨테이너 시작 시점에 USB 장치가 연결되지 않았음

**확인 방법:**
```bash
# 호스트에서 카메라 확인
v4l2-ctl --list-devices

# HD Pro Webcam C920 (usb-0000:06:00.0-1.4.2):
#     /dev/video6, /dev/video7
# HD Pro Webcam C920 (usb-0000:06:00.0-1.4.4):
#     /dev/video4, /dev/video5

# 컨테이너 내부 확인
docker exec adas_container ls /dev/video*
# /dev/video0 ~ /dev/video3만 존재  ← 문제!
```

**해결:**
```yaml
# compose.yaml
services:
  adas-dev:
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
      - /dev/ttyUSB0:/dev/ttyUSB0
      - /dev/video0:/dev/video0
      - /dev/video4:/dev/video4  # 후방 카메라
      - /dev/video5:/dev/video5
      - /dev/video6:/dev/video6  # 전방 카메라
      - /dev/video7:/dev/video7
```

```bash
# 컨테이너 재시작 (장치 재마운트)
cd /home/deepblue/target_projects/adas_env
docker compose down
docker compose up -d

# 확인
docker exec adas_container ls /dev/video*
# /dev/video0 ~ /dev/video7 전부 존재  ← 해결!
```

**참고:**
- `privileged: true`가 있어도 컨테이너 시작 후 연결된 USB 장치는 자동으로 마운트되지 않음
- USB 장치 연결/해제 시 컨테이너 재시작 필요

---

### Error 15: LiDAR Point Cloud Not Visible in RViz2

**증상:**
- RViz2에서 Fixed Frame을 base_link로 설정해도 LiDAR 포인트 클라우드가 보이지 않음
- Static TF Publisher는 추가했으나 여전히 데이터 없음

**에러 메시지:**
```
# RViz2에서 LaserScan 추가 시
Topic: /scan
Status: No messages received
```

**원인:**
- Static TF Publisher (base_link → laser)는 추가했으나 **RPLiDAR 드라이버 노드가 실행되지 않음**
- `track_launch.py`와 `mission_launch.py`에서 `rplidar_launch.py` 호출이 누락됨
- TF는 센서 **위치**만 정의, 드라이버는 실제 **센서 데이터** 발행

**개념 정리:**
```
Static TF Publisher:
  - 역할: base_link → laser 좌표 변환 정의
  - 발행: /tf_static 토픽
  - 예: "LiDAR는 로봇 중심에서 위로 10cm"

RPLiDAR Driver:
  - 역할: 물리적 센서에서 데이터 읽기
  - 발행: /scan 토픽 (sensor_msgs/LaserScan)
  - 예: "360도 거리 측정 데이터"

RViz2 LaserScan:
  - 필요: TF (위치) + /scan (데이터)
  - TF만 있으면: 좌표계만 표시, 데이터 없음
  - /scan만 있으면: "Frame [laser] does not exist" 에러
```

**확인 방법:**
```bash
# 1. TF 확인 (정상)
ros2 topic echo /tf_static | grep laser
# 출력: base_link → laser 변환 존재

# 2. /scan 토픽 확인 (문제!)
ros2 topic list | grep scan
# /scan 토픽 없음  ← 드라이버 미실행

# 3. RPLiDAR 노드 확인
ros2 node list | grep rplidar
# rplidarNode 없음  ← 드라이버 미실행
```

**해결:**

1. **track_launch.py 수정** (라인 271-290)
```python
# RPLiDAR launch 추가
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

2. **mission_launch.py 수정** (라인 295-314)
```python
# 주석 처리된 코드 활성화
# NOTE: Ensure rplidar_ros is running separately if needed
# 위 주석 삭제하고 rplidar_launch 추가

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
    ...
    lane_perception_launch,
    rplidar_launch,  # 주석 해제
    OpaqueFunction(function=launch_setup),
])
```

3. **재빌드**
```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select bringup
source install/setup.bash
```

4. **rplidar_ros 패키지 설치**

**방법 1: 현재 컨테이너에 수동 설치 (임시)**
```bash
# 런치 파일 실행 시 에러 발생:
# [ERROR] "package 'rplidar_ros' not found"

# rplidar_ros는 외부 패키지이므로 apt로 설치 필요
apt-get update
apt-get install -y ros-humble-rplidar-ros

# 설치 확인
source /opt/ros/humble/setup.bash
ros2 pkg list | grep rplidar
# 출력: rplidar_ros
```

**방법 2: Dockerfile 수정 (영구적 - 권장)**
```dockerfile
# /home/deepblue/target_projects/adas_env/Dockerfile
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
docker compose down && docker compose up -d
```

**패키지 구조 이해:**
- `rplidar_driver`: 우리가 만든 wrapper 패키지 (launch, config만 포함)
- `rplidar_ros`: SLAMTEC의 공식 ROS2 드라이버 (실제 실행 파일 포함)
- rplidar_launch.py는 rplidar_ros 패키지의 rplidar_node를 실행

**주의:**
- 수동 설치(방법 1)는 컨테이너 재시작 시 사라짐
- Dockerfile 수정(방법 2)은 영구적으로 유지됨

**검증:**
```bash
# 1. 시스템 실행
ros2 launch bringup track_launch.py

# 2. 다른 터미널에서 확인
# /scan 토픽 발행 확인
ros2 topic hz /scan
# 출력: average rate: 10.000

# TF 트리 확인
ros2 run tf2_tools view_frames
# frames.pdf 생성 → base_link → laser 확인

# 3. RViz2
rviz2
# Fixed Frame: base_link
# Add → LaserScan
# Topic: /scan
# → 빨간색 포인트 클라우드 정상 표시
```

**결과:**
- `/scan` 토픽 정상 발행 (10Hz)
- RViz2에서 LiDAR 포인트 클라우드 표시됨
- 장애물 감지 기능 정상 작동

**핵심 교훈:**
- TF (좌표 변환) ≠ 센서 데이터
- 센서 시각화 = TF + 센서 드라이버 (둘 다 필요!)

---
