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
