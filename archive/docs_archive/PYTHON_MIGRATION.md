# Python 노드 ROS1 → ROS2 변환 가이드

이 문서는 Python 노드를 ROS1에서 ROS2로 변환하는 상세 가이드입니다.

## 목차

- [개요](#개요)
- [변환 완료 노드](#변환-완료-노드)
- [기본 변환 패턴](#기본-변환-패턴)
- [상세 변환 예시](#상세-변환-예시)
- [나머지 노드 변환 계획](#나머지-노드-변환-계획)

---

## 개요

### 변환 현황

**완료:** 2개 노드
**남은 노드:** 16개 (YOLO 기반 노드 포함)

### 주요 변경사항

| 항목 | ROS1 | ROS2 |
|------|------|------|
| 모듈 | `rospy` | `rclpy` |
| 노드 클래스 | 일반 클래스 | `Node` 상속 필수 |
| 초기화 | `rospy.init_node()` | `rclpy.init()` + Node 생성 |
| 파라미터 | `rospy.get_param()` | `declare_parameter()` + `get_parameter()` |
| Publisher | `rospy.Publisher()` | `create_publisher()` |
| Subscriber | `rospy.Subscriber()` | `create_subscription()` |
| Timer | `rospy.Timer()` | `create_timer()` |
| 로깅 | `rospy.loginfo()` | `self.get_logger().info()` |
| 시간 | `rospy.Time.now()` | `self.get_clock().now()` |
| Spin | `rospy.spin()` | `rclpy.spin(node)` |

---

## 변환 완료 노드

### 1. lidar_obstacle_node.py

**파일 위치:** `src/decision/scripts/lidar_obstacle_node.py`

#### ROS1 버전
```python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

class LidarObstacleNode:
    def __init__(self) -> None:
        self.angle_min_deg = float(rospy.get_param("~angle_min_deg", 350.0))
        self.flag_pub = rospy.Publisher("/perception/obstacle_flag", Bool, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.loginfo("[lidar_obstacle] started")

def main() -> None:
    rospy.init_node("lidar_obstacle_node")
    LidarObstacleNode()
    rospy.spin()
```

#### ROS2 버전
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

class LidarObstacleNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_obstacle_node')

        # 파라미터 선언
        self.declare_parameter('angle_min_deg', 350.0)

        # 파라미터 가져오기
        self.angle_min_deg = self.get_parameter('angle_min_deg').get_parameter_value().double_value

        # Publisher
        self.flag_pub = self.create_publisher(Bool, '/perception/obstacle_flag', 1)

        # Subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 1)

        self.get_logger().info('[lidar_obstacle] started')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 2. decision_node_2026.py

**파일 위치:** `src/decision/scripts/decision_node_2026.py`

#### 주요 변경사항

1. **타이머 콜백**
```python
# ROS1
rospy.Timer(rospy.Duration(0.05), self.timer_cb)
def timer_cb(self, _event) -> None:
    pass

# ROS2
self.timer = self.create_timer(0.05, self.timer_cb)
def timer_cb(self) -> None:
    pass
```

2. **시간 처리**
```python
# ROS1
self.lane_stamp = rospy.Time.now()
age = (rospy.Time.now() - self.lane_stamp).to_sec()

# ROS2
from rclpy.time import Time
self.lane_stamp: Optional[Time] = None
self.lane_stamp = self.get_clock().now()
age = (self.get_clock().now() - self.lane_stamp).nanoseconds / 1e9
```

---

## 기본 변환 패턴

### 1. 노드 클래스 정의

#### ROS1
```python
class MyNode:
    def __init__(self):
        # 초기화
        pass
```

#### ROS2
```python
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')  # 노드 이름 지정
        # 초기화
```

### 2. 파라미터

#### ROS1
```python
self.param1 = rospy.get_param("~param1", 10)
self.param2 = float(rospy.get_param("~param2", 1.5))
self.param3 = bool(rospy.get_param("~param3", True))
```

#### ROS2
```python
# 먼저 선언
self.declare_parameter('param1', 10)
self.declare_parameter('param2', 1.5)
self.declare_parameter('param3', True)

# 그 다음 가져오기
self.param1 = self.get_parameter('param1').get_parameter_value().integer_value
self.param2 = self.get_parameter('param2').get_parameter_value().double_value
self.param3 = self.get_parameter('param3').get_parameter_value().bool_value
```

**파라미터 타입별 메서드:**
- `integer_value` - int
- `double_value` - float
- `bool_value` - bool
- `string_value` - str

### 3. Publisher

#### ROS1
```python
self.pub = rospy.Publisher('/topic', MsgType, queue_size=10)
self.pub.publish(msg)
```

#### ROS2
```python
self.pub = self.create_publisher(MsgType, '/topic', 10)
self.pub.publish(msg)
```

### 4. Subscriber

#### ROS1
```python
rospy.Subscriber('/topic', MsgType, self.callback, queue_size=10)

def callback(self, msg: MsgType) -> None:
    pass
```

#### ROS2
```python
self.sub = self.create_subscription(
    MsgType,
    '/topic',
    self.callback,
    10
)

def callback(self, msg: MsgType) -> None:
    pass
```

### 5. Timer

#### ROS1
```python
rospy.Timer(rospy.Duration(0.1), self.timer_callback)

def timer_callback(self, event) -> None:
    pass
```

#### ROS2
```python
self.timer = self.create_timer(0.1, self.timer_callback)

def timer_callback(self) -> None:
    pass
```

### 6. 로깅

#### ROS1
```python
rospy.logdebug("Debug message")
rospy.loginfo("Info message")
rospy.logwarn("Warning message")
rospy.logerr("Error message")
rospy.logfatal("Fatal message")
```

#### ROS2
```python
self.get_logger().debug("Debug message")
self.get_logger().info("Info message")
self.get_logger().warning("Warning message")
self.get_logger().error("Error message")
self.get_logger().fatal("Fatal message")
```

### 7. 시간 처리

#### ROS1
```python
import rospy
from rospy import Time

now = rospy.Time.now()
duration = rospy.Duration(1.0)
timestamp = Time()
timestamp.secs = 100
timestamp.nsecs = 500
```

#### ROS2
```python
from rclpy.time import Time
from rclpy.duration import Duration

now = self.get_clock().now()
duration = Duration(seconds=1.0)
# Time은 nanoseconds로 표현
timestamp = Time(nanoseconds=100_000_500)
```

### 8. main() 함수

#### ROS1
```python
def main() -> None:
    rospy.init_node("my_node")
    MyNode()
    rospy.spin()

if __name__ == "__main__":
    main()
```

#### ROS2
```python
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## 상세 변환 예시

### 예시 1: 간단한 Publisher 노드

#### ROS1
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class SimplePublisher:
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate", 10)
        self.pub = rospy.Publisher("/hello", String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate_hz), self.timer_cb)
        rospy.loginfo("SimplePublisher started")

    def timer_cb(self, event):
        msg = String()
        msg.data = "Hello World"
        self.pub.publish(msg)

def main():
    rospy.init_node("simple_publisher")
    SimplePublisher()
    rospy.spin()

if __name__ == "__main__":
    main()
```

#### ROS2
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        self.declare_parameter('rate', 10)
        self.rate_hz = self.get_parameter('rate').get_parameter_value().integer_value

        self.pub = self.create_publisher(String, '/hello', 10)
        self.timer = self.create_timer(1.0/self.rate_hz, self.timer_cb)

        self.get_logger().info('SimplePublisher started')

    def timer_cb(self):
        msg = String()
        msg.data = 'Hello World'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### 예시 2: Subscriber + Publisher 노드

#### ROS1
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

class Processor:
    def __init__(self):
        self.gain = float(rospy.get_param("~gain", 2.0))
        self.sub = rospy.Subscriber("/input", Float32, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/output", Float32, queue_size=1)
        rospy.loginfo("Processor ready, gain=%.2f", self.gain)

    def callback(self, msg: Float32):
        output = Float32()
        output.data = msg.data * self.gain
        self.pub.publish(output)
        rospy.logdebug("Processed: %.2f -> %.2f", msg.data, output.data)

def main():
    rospy.init_node("processor")
    Processor()
    rospy.spin()

if __name__ == "__main__":
    main()
```

#### ROS2
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Processor(Node):
    def __init__(self):
        super().__init__('processor')

        self.declare_parameter('gain', 2.0)
        self.gain = self.get_parameter('gain').get_parameter_value().double_value

        self.sub = self.create_subscription(Float32, '/input', self.callback, 1)
        self.pub = self.create_publisher(Float32, '/output', 1)

        self.get_logger().info(f'Processor ready, gain={self.gain:.2f}')

    def callback(self, msg: Float32):
        output = Float32()
        output.data = msg.data * self.gain
        self.pub.publish(output)
        self.get_logger().debug(f'Processed: {msg.data:.2f} -> {output.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = Processor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## 나머지 노드 변환 계획

### 남은 Python 노드 목록

#### 1. 드라이버 노드 (2개)
- `arduino_driver/scripts/arduino_bridge_node.py` - 시리얼 통신
- `ultrasonic_driver/scripts/ultrasonic_processor_node.py` - 센서 처리

**특이사항:** pyserial 라이브러리 사용, 스레드 처리

#### 2. Decision 노드 (2개)
- `decision/scripts/decision_node.py` - 기본 decision 로직
- `decision/scripts/decision_node_ai.py` - AI 기반 decision

**특이사항:** 타이머 기반, 센서 융합

#### 3. Perception 노드 - 차선 (3개)
- `perception_pkg/scripts/lane_tracking_node.py`
- `perception_pkg/scripts/lane_marking_node.py`
- `perception_pkg/scripts/obstacle_detection_node.py`

**특이사항:** OpenCV 사용, 이미지 처리

#### 4. Perception 노드 - YOLO (3개)
- `perception_pkg/scripts/speed_sign_node.py` - 속도 표지판 인식
- `perception_pkg/scripts/traffic_light_node.py` - 신호등 인식
- `perception_pkg/scripts/traffic_light_color_node.py` - 신호등 색상 분류

**특이사항:** YOLO 모델 로딩, PyTorch/ultralytics 사용

#### 5. 디버그/모니터링 (2개)
- `common/scripts/ultrasonic_monitor.py`
- `common/scripts/lidar_monitor.py`

**특이사항:** 시각화, matplotlib 사용 가능성

### 변환 우선순위

1. **높음**: decision 노드들 (decision_node.py, decision_node_ai.py)
2. **중간**: 드라이버 노드들 (arduino_bridge_node.py, ultrasonic_processor_node.py)
3. **낮음**: YOLO 노드들 (Phase 8 완료 후)
4. **선택**: 디버그 노드들 (필요시)

### YOLO 노드 변환 시 추가 고려사항

1. **ultralytics 패키지**: ROS2에서도 동일하게 사용 가능
```python
# ROS1 / ROS2 공통
from ultralytics import YOLO
model = YOLO('path/to/model.pt')
```

2. **cv_bridge 사용법**: ROS2도 동일
```python
# ROS2
from cv_bridge import CvBridge
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
```

3. **모델 경로**: 패키지 경로 찾기 방법 변경
```python
# ROS1
from rospkg import RosPack
rp = RosPack()
pkg_path = rp.get_path('perception_pkg')

# ROS2
from ament_index_python.packages import get_package_share_directory
pkg_path = get_package_share_directory('perception_pkg')
```

---

## 체크리스트

변환 시 확인할 사항:

- [ ] `rospy` → `rclpy` 임포트 변경
- [ ] `Node` 클래스 상속
- [ ] `super().__init__('node_name')` 호출
- [ ] 파라미터: `declare_parameter()` → `get_parameter()`
- [ ] Publisher/Subscriber: `create_*()` 메서드 사용
- [ ] 타이머: `create_timer()` 사용, 콜백 인자 제거
- [ ] 로깅: `self.get_logger().*()` 사용
- [ ] 시간: `self.get_clock().now()` 사용
- [ ] main() 함수: try-finally 구조로 정리
- [ ] 메시지 생성: 명시적으로 객체 생성 후 publish

---

## 참고 자료

- [ROS2 Python Client Library (rclpy) API](https://docs.ros2.org/latest/api/rclpy/)
- [ROS1 to ROS2 Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)
- [ROS2 Python Programming](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
