# ROS1 → ROS2 API 변경사항

이 문서는 ROS1 Noetic과 ROS2 Humble 간의 주요 API 차이점을 정리합니다.

## 목차

- [헤더 파일](#헤더-파일)
- [노드 생성 및 관리](#노드-생성-및-관리)
- [파라미터](#파라미터)
- [Publisher & Subscriber](#publisher--subscriber)
- [타이머](#타이머)
- [로깅](#로깅)
- [시간 처리](#시간-처리)
- [메시지 타입](#메시지-타입)

---

## 헤더 파일

### 기본 헤더

| ROS1 | ROS2 |
|------|------|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |

### 메시지 헤더

| ROS1 | ROS2 |
|------|------|
| `#include <std_msgs/String.h>` | `#include <std_msgs/msg/string.hpp>` |
| `#include <std_msgs/Float32.h>` | `#include <std_msgs/msg/float32.hpp>` |
| `#include <std_msgs/Bool.h>` | `#include <std_msgs/msg/bool.hpp>` |
| `#include <sensor_msgs/Image.h>` | `#include <sensor_msgs/msg/image.hpp>` |
| `#include <sensor_msgs/LaserScan.h>` | `#include <sensor_msgs/msg/laser_scan.hpp>` |
| `#include <geometry_msgs/Twist.h>` | `#include <geometry_msgs/msg/twist.hpp>` |
| `#include <ackermann_msgs/AckermannDrive.h>` | `#include <ackermann_msgs/msg/ackermann_drive.hpp>` |

**규칙:**
- ROS1: `<package/MessageType.h>`
- ROS2: `<package/msg/message_type.hpp>` (snake_case)

---

## 노드 생성 및 관리

### 노드 클래스 정의

#### ROS1
```cpp
class MyNode {
public:
    MyNode() : nh_(), pnh_("~") {
        // 초기화
    }

private:
    ros::NodeHandle nh_;         // 전역 네임스페이스
    ros::NodeHandle pnh_;        // 프라이빗 네임스페이스
};
```

#### ROS2
```cpp
class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {  // 노드 이름 지정
        // 초기화
    }

private:
    // NodeHandle 불필요 (Node 클래스 자체가 핸들)
};
```

**주요 차이점:**
- ROS2는 `rclcpp::Node` 상속 필수
- NodeHandle 개념 제거 (Node 클래스가 통합)
- 생성자에서 노드 이름 지정

### main() 함수

#### ROS1
```cpp
int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    MyNode node;
    ros::spin();
    return 0;
}
```

#### ROS2
```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

**주요 차이점:**
- `ros::init()` → `rclcpp::init()`
- 노드를 `shared_ptr`로 생성
- `rclcpp::shutdown()` 명시적 호출 권장

---

## 파라미터

### 파라미터 읽기

#### ROS1
```cpp
// 프라이빗 파라미터
std::string topic = pnh_.param<std::string>("topic", "/default");
int rate = pnh_.param<int>("rate", 10);
double gain = pnh_.param<double>("gain", 1.0);

// 전역 파라미터
ros::param::get("/global/param", value);
```

#### ROS2
```cpp
// 파라미터 선언 (필수!)
this->declare_parameter("topic", "/default");
this->declare_parameter("rate", 10);
this->declare_parameter("gain", 1.0);

// 파라미터 읽기
std::string topic = this->get_parameter("topic").as_string();
int rate = this->get_parameter("rate").as_int();
double gain = this->get_parameter("gain").as_double();
```

**주요 차이점:**
1. **선언 필수**: `declare_parameter()` 먼저 호출해야 함
2. **타입 변환**: `.as_string()`, `.as_int()`, `.as_double()` 등 명시적 변환
3. **프라이빗 네임스페이스**: 자동으로 노드 이름 아래에 위치

### 파라미터 업데이트

#### ROS1
```cpp
// 실시간 파라미터 업데이트 (dynamic_reconfigure)
// 별도 설정 파일 및 콜백 필요
```

#### ROS2
```cpp
// 파라미터 콜백 설정
auto param_callback = [this](const std::vector<rclcpp::Parameter>& params) {
    for (const auto& param : params) {
        if (param.get_name() == "gain") {
            gain_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Gain updated to: %.2f", gain_);
        }
    }
    return rcl_interfaces::msg::SetParametersResult();
};

param_callback_handle_ = this->add_on_set_parameters_callback(param_callback);
```

**ROS2 장점:**
- dynamic_reconfigure 별도 설정 불필요
- 표준 파라미터 시스템에 통합

---

## Publisher & Subscriber

### Publisher 생성

#### ROS1
```cpp
ros::Publisher pub = nh_.advertise<std_msgs::String>("topic", 10);
```

#### ROS2
```cpp
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub =
    this->create_publisher<std_msgs::msg::String>("topic", 10);

// QoS 지정 (ROS2만 가능)
auto qos = rclcpp::QoS(10).reliable();
pub = this->create_publisher<std_msgs::msg::String>("topic", qos);
```

### Subscriber 생성

#### ROS1
```cpp
ros::Subscriber sub = nh_.subscribe("topic", 10,
                                    &MyNode::callback, this);

void MyNode::callback(const std_msgs::StringConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}
```

#### ROS2
```cpp
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub =
    this->create_subscription<std_msgs::msg::String>(
        "topic", 10,
        std::bind(&MyNode::callback, this, std::placeholders::_1));

void MyNode::callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}
```

**주요 차이점:**
1. **포인터 타입**: `ConstPtr` → `SharedPtr`
2. **콜백 바인딩**: `std::bind()` 사용
3. **QoS 지원**: ROS2에서 QoS 정책 명시 가능

### 메시지 발행

#### ROS1
```cpp
std_msgs::String msg;
msg.data = "hello";
pub.publish(msg);
```

#### ROS2
```cpp
std_msgs::msg::String msg;
msg.data = "hello";
pub->publish(msg);  // -> 연산자 사용 (SharedPtr이므로)
```

---

## 타이머

### 타이머 생성

#### ROS1
```cpp
ros::Timer timer = nh.createTimer(ros::Duration(0.1),
                                  &MyNode::timerCallback, this);

void MyNode::timerCallback(const ros::TimerEvent& event) {
    ROS_INFO("Timer triggered");
}
```

#### ROS2
```cpp
rclcpp::TimerBase::SharedPtr timer =
    this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MyNode::timerCallback, this));

void MyNode::timerCallback() {
    RCLCPP_INFO(this->get_logger(), "Timer triggered");
}
```

**주요 차이점:**
1. **시간 단위**: `ros::Duration` → `std::chrono`
2. **타이머 타입**:
   - `create_wall_timer()`: 시스템 시간 기반 (일반적으로 사용)
   - `create_timer()`: ROS 시간 기반
3. **콜백**: TimerEvent 인자 없음

### 타이머 타입 선택

```cpp
// Wall timer: 실시간 시스템 시간 기반
timer_ = this->create_wall_timer(std::chrono::milliseconds(100), callback);

// ROS timer: ROS 시뮬레이션 시간 지원
timer_ = this->create_timer(std::chrono::milliseconds(100), callback);
```

**권장:** 제어 루프는 `create_wall_timer()` 사용 (더 정확)

---

## 로깅

### 기본 로깅

| ROS1 | ROS2 |
|------|------|
| `ROS_DEBUG("msg")` | `RCLCPP_DEBUG(this->get_logger(), "msg")` |
| `ROS_INFO("msg")` | `RCLCPP_INFO(this->get_logger(), "msg")` |
| `ROS_WARN("msg")` | `RCLCPP_WARN(this->get_logger(), "msg")` |
| `ROS_ERROR("msg")` | `RCLCPP_ERROR(this->get_logger(), "msg")` |
| `ROS_FATAL("msg")` | `RCLCPP_FATAL(this->get_logger(), "msg")` |

### 포맷 로깅

#### ROS1
```cpp
ROS_INFO("Value: %d, Name: %s", value, name.c_str());
```

#### ROS2
```cpp
RCLCPP_INFO(this->get_logger(), "Value: %d, Name: %s", value, name.c_str());
```

### Throttle 로깅 (주기적 로깅)

#### ROS1
```cpp
ROS_INFO_THROTTLE(1.0, "This prints once per second");
```

#### ROS2
```cpp
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                     "This prints once per second");
// 시간 단위: 밀리초
```

**주요 차이점:**
- ROS2는 Clock 객체 명시 필요
- 시간 단위: 초(ROS1) → 밀리초(ROS2)

---

## 시간 처리

### 현재 시간 가져오기

#### ROS1
```cpp
ros::Time now = ros::Time::now();
double seconds = now.toSec();
```

#### ROS2
```cpp
rclcpp::Time now = this->now();
double seconds = now.seconds();
```

### Duration 생성

#### ROS1
```cpp
ros::Duration d(1.0);  // 1초
ros::Duration d = ros::Duration(0.1);  // 100ms
```

#### ROS2
```cpp
rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
auto d = std::chrono::seconds(1);
auto d = std::chrono::milliseconds(100);
```

### 시간 연산

#### ROS1
```cpp
ros::Time start = ros::Time::now();
ros::Duration elapsed = ros::Time::now() - start;

if (elapsed.toSec() > 1.0) {
    // 1초 경과
}
```

#### ROS2
```cpp
rclcpp::Time start = this->now();
rclcpp::Duration elapsed = this->now() - start;

if (elapsed.seconds() > 1.0) {
    // 1초 경과
}
```

---

## 메시지 타입

### 기본 메시지 타입 변환

| ROS1 | ROS2 |
|------|------|
| `std_msgs::String` | `std_msgs::msg::String` |
| `std_msgs::Float32` | `std_msgs::msg::Float32` |
| `std_msgs::Bool` | `std_msgs::msg::Bool` |
| `std_msgs::Header` | `std_msgs::msg::Header` |
| `std_msgs::Float32MultiArray` | `std_msgs::msg::Float32MultiArray` |

### 포인터 타입 변환

| ROS1 | ROS2 |
|------|------|
| `std_msgs::StringConstPtr` | `std_msgs::msg::String::SharedPtr` |
| `sensor_msgs::ImageConstPtr` | `sensor_msgs::msg::Image::SharedPtr` |

### 헤더 접근 (동일)

```cpp
// ROS1과 ROS2 모두 동일
msg->header.stamp
msg->header.frame_id
msg->header.seq  // ROS2에서는 제거됨
```

---

## QoS (Quality of Service)

### ROS1
- QoS 개념 없음
- 큐 크기만 지정 가능

### ROS2
QoS 프로필로 통신 특성 제어 가능

#### 주요 QoS 설정

```cpp
// 1. Reliable (기본값): 모든 메시지 전달 보장
auto qos_reliable = rclcpp::QoS(10).reliable();

// 2. Best Effort: 네트워크 혼잡 시 메시지 스킵 (센서 데이터 권장)
auto qos_best_effort = rclcpp::QoS(10).best_effort();

// 3. History: 저장할 메시지 개수
auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // 최근 10개
auto qos = rclcpp::QoS(rclcpp::KeepAll());     // 모두 저장

// 4. Durability: 늦게 구독한 노드에 과거 메시지 전달
auto qos = rclcpp::QoS(10).transient_local();  // 과거 메시지 전달
auto qos = rclcpp::QoS(10).volatile_durability();  // 현재 메시지만
```

#### 사용 예시

**센서 데이터 (지연시간 최소화):**
```cpp
auto qos = rclcpp::QoS(1).best_effort();
sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image", qos, callback);
```

**제어 명령 (신뢰성 우선):**
```cpp
auto qos = rclcpp::QoS(10).reliable();
pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "/cmd", qos);
```

---

## 빌드 시스템

### CMakeLists.txt

#### ROS1
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node ${catkin_LIBRARIES})
```

#### ROS2
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node
  rclcpp
  std_msgs
)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### package.xml

#### ROS1 (format 2)
```xml
<package format="2">
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
</package>
```

#### ROS2 (format 3)
```xml
<package format="3">
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 명령어 비교

| 작업 | ROS1 | ROS2 |
|------|------|------|
| 빌드 | `catkin_make` 또는 `catkin build` | `colcon build` |
| 소싱 | `source devel/setup.bash` | `source install/setup.bash` |
| 노드 실행 | `rosrun pkg node` | `ros2 run pkg node` |
| 런치 | `roslaunch pkg file.launch` | `ros2 launch pkg file.py` |
| 토픽 목록 | `rostopic list` | `ros2 topic list` |
| 토픽 에코 | `rostopic echo /topic` | `ros2 topic echo /topic` |
| 노드 목록 | `rosnode list` | `ros2 node list` |
| 파라미터 목록 | `rosparam list` | `ros2 param list` |
| 파라미터 설정 | `rosparam set /param value` | `ros2 param set /node param value` |

---

## 요약

### ROS2의 주요 개선사항

1. **QoS 시스템**: 통신 품질 세밀 제어 가능
2. **타입 안전성**: 컴파일 시점에 더 많은 오류 검출
3. **파라미터 시스템**: 동적 업데이트 기본 지원
4. **멀티스레딩**: Executor 개념으로 더 나은 스레드 제어
5. **타이밍**: `create_wall_timer()`로 더 정확한 주기 제어

### 마이그레이션 체크리스트

- [ ] 헤더 파일: `<pkg/Msg.h>` → `<pkg/msg/msg.hpp>`
- [ ] 클래스: `rclcpp::Node` 상속
- [ ] NodeHandle 제거
- [ ] 파라미터: `declare_parameter()` 추가
- [ ] 콜백: `std::bind()` 사용
- [ ] 포인터: `ConstPtr` → `SharedPtr`
- [ ] 로깅: `ROS_*` → `RCLCPP_*`
- [ ] 시간: `ros::Time` → `rclcpp::Time`
- [ ] 타이머: `std::chrono` 사용
- [ ] 빌드: `catkin` → `ament_cmake`
- [ ] QoS: 센서/제어별 최적 프로필 적용
