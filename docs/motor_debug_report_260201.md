# 모터 구동 문제 디버깅 리포트

## 증상

- `ros2 launch bringup track_launch.py` 실행 시 모터가 전혀 동작하지 않음
- 차선 인식, LiDAR 등 다른 센서는 정상 동작
- Arduino IDE에서 직접 시리얼 모니터로 명령 전송 시에는 모터 정상 동작
- ROS2를 통해서만 모터가 동작하지 않는 상태

## 원인 분석 과정

### 1단계: ROS2 토픽 확인

```bash
ros2 topic echo /arduino/cmd --once
```

- 결과: `speed: 1.0, steering_angle: -0.106...`
- **결론:** Decision 노드는 정상적으로 명령을 발행 중

### 2단계: 노드 연결 확인

```bash
ros2 node info /arduino_bridge
```

- 결과: `/arduino/cmd` 토픽 구독 확인
- **결론:** arduino_bridge 노드가 토픽을 수신하고 있음

### 3단계: 시리얼 출력 확인

```bash
sudo cat /dev/ttyACM0
```

- 결과: **출력 없음**
- **결론:** C++ arduino_bridge가 토픽은 수신하지만 시리얼에 데이터를 쓰지 않음

### 4단계: 시리얼 직접 테스트

```bash
python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
ser.write(b'V:128,S:90\n')
print('sent: V:128,S:90')
time.sleep(1)
data = ser.read(ser.in_waiting or 1)
print('recv:', data)
ser.close()
"
```

- 결과: `CMD:128,90` 응답 수신, **모터 정상 동작**
- **결론:** Arduino 하드웨어/펌웨어 정상. 문제는 arduino_bridge 노드에 있음

## 근본 원인 (2가지)

### 원인 1: C++ arduino_bridge의 boost::asio 시리얼 쓰기 실패

C++ `arduino_bridge_node.cpp`에서 `boost::asio::write()` 호출이 실제 시리얼 포트에 데이터를 쓰지 못하는 문제가 있었다. 에러 로그도 출력되지 않아 원인 파악이 어려웠다.

**파일:** `src/drivers/arduino_driver/src/arduino_bridge_node.cpp` (96-100행)

```cpp
// 코드 자체는 정상이나 boost::asio가 실제로 데이터를 쓰지 않음
int pwm = speedToPwm(msg->speed);
int servo = steerToServo(msg->steering_angle);
std::ostringstream oss;
oss << "V:" << pwm << ",S:" << servo << "\n";
boost::asio::write(*serial_port_, boost::asio::buffer(oss.str()));
```

### 원인 2: Arduino 리셋 대기 시간 부재

`serial.Serial()` 또는 `boost::asio::serial_port::open()` 호출 시 DTR 라인이 토글되면서 Arduino가 하드웨어 리셋된다. Arduino 부팅에 약 2초가 소요되는데, 이 시간 동안 전송된 명령은 모두 유실된다.

직접 테스트에서 `time.sleep(2)`를 넣었을 때 동작한 이유가 바로 이것이다.

## 해결 방법

### 해결 1: arduino_bridge를 항상 Python으로 실행

C++ boost::asio 시리얼 문제를 우회하기 위해, `track_launch.py`에서 `use_cpp` 설정과 관계없이 항상 Python arduino_bridge를 사용하도록 변경했다.

**파일:** `src/bringup/launch/track_launch.py`

```python
# 변경 전: use_cpp 분기
if use_cpp:
    arduino_node = Node(
        package='arduino_driver',
        executable='arduino_bridge_node',  # C++
        ...
    )
else:
    arduino_node = Node(
        package='arduino_driver',
        executable='arduino_bridge_node.py',  # Python
        ...
    )

# 변경 후: 항상 Python 사용
arduino_node = Node(
    package='arduino_driver',
    executable='arduino_bridge_node.py',
    name='arduino_bridge',
    output='screen',
    parameters=[...]
)
```

### 해결 2: Python bridge에 Arduino 리셋 대기 추가

**파일:** `src/drivers/arduino_driver/scripts/arduino_bridge_node.py`

```python
# 시리얼 포트 열기 (DTR 토글로 Arduino 리셋 발생)
self.serial = serial.Serial(self.port, self.baudrate, timeout=0.01)

# Arduino 리셋 후 부팅 대기 (2초)
self.get_logger().info("[arduino_bridge] Arduino 리셋 대기 중 (2초)...")
import time
time.sleep(2)
# 부팅 메시지 비우기
if self.serial.in_waiting:
    self.serial.read(self.serial.in_waiting)
self.get_logger().info("[arduino_bridge] Arduino 준비 완료")
```

### 해결 3: Python bridge를 rospy → rclpy로 마이그레이션

기존 Python bridge는 ROS1(`rospy`)을 사용하여 ROS2 환경에서 실행 불가했다. `rclpy`로 전면 재작성했다.

주요 변경점:
- `rospy` → `rclpy` / `rclpy.node.Node`
- `rospy.get_param("~port")` → `self.declare_parameter()` + `self.get_parameter()`
- `rospy.Subscriber()` → `self.create_subscription()`
- `rospy.Publisher()` → `self.create_publisher()`
- `rospy.Rate()` + 루프 → `self.create_timer()` 콜백
- Legacy 명령 모드 제거, 연속 모드(`V:pwm,S:servo`)만 사용
- `threading.Lock` 추가로 읽기/쓰기 시리얼 충돌 방지

## 수정된 파일 목록

| 파일 | 변경 내용 |
|------|----------|
| `src/drivers/arduino_driver/scripts/arduino_bridge_node.py` | rospy→rclpy 마이그레이션, Arduino 리셋 대기 2초 추가 |
| `src/bringup/launch/track_launch.py` | arduino_bridge를 항상 Python으로 실행하도록 변경 |
| `src/drivers/arduino_driver/config/arduino.yaml` | `use_legacy_cmd` 파라미터 제거 (연속 모드만 사용) |
| `arduino_firmware/ackermann_drive_firmware.ino` | ROS2 V:pwm,S:servo 프로토콜 대응 펌웨어 |

## 디버깅 명령어 모음

```bash
# 1. ROS2 명령 발행 확인
ros2 topic echo /arduino/cmd --once

# 2. 노드 실행 확인
ros2 node list | grep arduino

# 3. 시리얼 직접 테스트 (launch 중지 후)
python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
ser.write(b'V:128,S:90\n')
print('sent:', ser.read(ser.in_waiting or 1))
ser.close()
"

# 4. 모터 직접 제어 (decision_node 우회)
ros2 topic pub /arduino/cmd ackermann_msgs/msg/AckermannDrive "{speed: 1.0, steering_angle: 0.0}" -r 10

# 5. launch 로그에서 에러 확인
ros2 launch bringup track_launch.py 2>&1 | grep -i "arduino\|error\|traceback"
```

## 교훈

1. **시리얼 통신은 직접 테스트로 분리 검증:** ROS2 노드를 거치지 않고 `pyserial`로 직접 쓰기/읽기를 테스트하면 문제 범위를 빠르게 좁힐 수 있다.
2. **Arduino DTR 리셋 고려:** 시리얼 포트를 열 때 Arduino가 리셋되므로, 부팅 대기(2초)가 필수다.
3. **C++/Python 혼합 전략:** 대부분 C++ 노드를 사용하되, 시리얼 통신처럼 Python이 더 안정적인 부분은 Python을 사용하는 혼합 전략이 효과적이다.
