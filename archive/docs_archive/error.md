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

## 2026-02-01

### Error 16: C++ Arduino Bridge Silent Write Failure (boost::asio)

**증상:**
- `ros2 topic echo /arduino/cmd`에서 명령은 정상 발행됨
- Arduino에서 모터가 동작하지 않음
- 에러 메시지 없음 (silent failure)
- `sudo cat /dev/ttyACM0`에서 아무 데이터도 나오지 않음

**원인:**
- C++ arduino_bridge_node에서 boost::asio::write()가 시리얼 포트에 데이터를 쓰지 못함
- 에러가 발생하지 않아 디버깅 어려움
- 근본 원인 미확인 (boost::asio 설정 또는 Docker 환경 관련 추정)

**확인 방법:**
```bash
# 1. 토픽 명령 확인 (정상 발행됨)
ros2 topic echo /arduino/cmd
# drive: speed: 0.5, steering_angle: 0.0  <- 명령 있음

# 2. 시리얼 출력 확인 (데이터 없음)
sudo cat /dev/ttyACM0
# (아무것도 출력 안됨)  <- 문제!

# 3. Python으로 직접 시리얼 테스트 (정상 동작)
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
time.sleep(2)
s.write(b'V:128,S:90\n')
# Arduino에서 CMD:128,90 출력됨  <- pyserial은 정상
"
```

**해결:**
- C++ bridge 대신 Python bridge (pyserial) 사용
- `track_launch.py`에서 Arduino bridge를 항상 Python으로 실행하도록 변경

```python
# track_launch.py
# Arduino bridge node (항상 Python 사용 - C++ boost::asio 시리얼 문제)
arduino_node = Node(
    package='arduino_driver',
    executable='arduino_bridge_node.py',
    name='arduino_bridge',
    ...
)
```

**참고:** 다른 노드(decision, lane_tracking 등)는 C++ 정상 동작

---

### Error 17: Arduino DTR Reset - 초기 명령 손실

**증상:**
- Python arduino_bridge 실행 직후 첫 몇 초간 명령이 Arduino에 전달되지 않음
- 그 이후에는 정상 동작

**원인:**
- pyserial이 시리얼 포트를 열 때 DTR(Data Terminal Ready) 핀이 토글됨
- Arduino는 DTR 핀 변화 시 자동으로 리셋됨
- 리셋 후 부트로더 실행 -> 약 2초 소요
- 부팅 중 수신한 시리얼 데이터는 무시됨

**해결:**
```python
# arduino_bridge_node.py
self.serial = serial.Serial(self.port, self.baudrate, timeout=0.01)

# Arduino 리셋 후 부팅 대기 (2초)
self.get_logger().info("[arduino_bridge] Arduino 리셋 대기 중 (2초)...")
import time
time.sleep(2)

# 부팅 중 Arduino가 보낸 쓰레기 데이터 플러시
if self.serial.in_waiting:
    self.serial.read(self.serial.in_waiting)

self.get_logger().info("[arduino_bridge] Arduino 준비 완료")
```

---

### Error 18: Python 노드 rospy ModuleNotFoundError

**에러 메시지:**
```
ModuleNotFoundError: No module named 'rospy'
```

**영향 파일 (12개):**
- `scripts/lane_tracking_node.py`
- `scripts/lane_marking_node.py`
- `scripts/speed_sign_node.py`
- `scripts/traffic_light_node.py`
- `scripts/traffic_light_color_node.py`
- `scripts/obstacle_detection_node.py`
- `perception_pkg/parking_line_node.py`
- 기타 Python 노드

**원인:**
- ROS2 Humble 환경에서 rospy (ROS1 라이브러리) 사용
- ROS2에서는 rclpy를 사용해야 함

**해결:**
- `use_cpp:=true` (기본값)으로 실행하면 C++ 노드가 사용되므로 문제 없음
- Arduino bridge만 Python으로 마이그레이션 완료 (rclpy)
- 나머지 12개 노드는 C++ 대체 사용 중

```bash
# 정상 실행 (C++ 노드 사용)
ros2 launch bringup track_launch.py

# 에러 발생 (Python 노드 사용 시)
ros2 launch bringup track_launch.py use_cpp:=false
# -> rospy 에러로 Python 노드 크래시
```

---

### Error 19: test_mode에서 speed: 0.0 (모터 안 돌아감)

**증상:**
```bash
ros2 topic echo /arduino/cmd
# drive: speed: 0.0, steering_angle: 0.0  <- 속도가 0
```

**원인 1: use_cpp:=false 사용 시**
- Python decision_node도 rospy 사용 -> 크래시
- 크래시된 decision_node가 speed를 발행하지 않음

**원인 2: Arduino bridge가 C++ (boost::asio)**
- 명령은 발행되지만 시리얼에 write 안됨

**해결:**
```bash
# 올바른 실행 방법 (C++ 노드 + Python Arduino bridge)
ros2 launch bringup track_launch.py test_mode:=true

# 잘못된 실행 방법
ros2 launch bringup track_launch.py test_mode:=true use_cpp:=false
```

---

### Error 20: rviz2 "Frame [map] does not exist"

**에러 메시지:**
```
[rviz2]: Transform [sender=unknown_publisher]
For frame [map]: Fixed Frame [map] does not exist
```

**원인:**
- rviz2의 Fixed Frame이 `map`으로 설정되어 있으나, SLAM을 사용하지 않아 map 프레임이 존재하지 않음

**해결:**
- rviz2에서 Fixed Frame을 `base_link`로 변경
- Global Options -> Fixed Frame -> `base_link`

---

### Error 21: rviz2에서 카메라 Image 안 보임

**증상:**
- rviz2에서 카메라 영상이 보이지 않음
- Camera display를 추가했지만 "No Image" 표시

**원인:**
- `Camera` display는 카메라 캘리브레이션 (camera_info) 토픽이 필요
- 현재 camera_info가 설정되지 않아 Camera display 사용 불가

**해결:**
- `Camera` 대신 `Image` display 사용
- Add -> By display type -> Image
- Topic: `/camera/front/image` (또는 `/lane_overlay`, `/yolo/overlay`)

---

### Error 22: git add 멈춤 (rosbag2 대용량 파일)

**증상:**
- `git add .` 실행 시 오래 걸리거나 멈춤

**원인:**
- `rosbag2_*/` 디렉토리에 대용량 녹화 파일 존재
- git이 모든 파일을 인덱싱하려다 멈춤

**해결:**
```bash
# .gitignore에 추가
rosbag2_*/

# 이미 추적 중인 파일 제거
git rm -r --cached rosbag2_*/
```

---
