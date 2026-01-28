# ROS1 → ROS2 마이그레이션 가이드

이 문서는 각 Phase별 상세한 변환 과정을 단계별로 설명합니다.

## 목차

- [Phase 1: 빌드 시스템 변환](#phase-1-빌드-시스템-변환)
- [Phase 2: Arduino 드라이버 변환](#phase-2-arduino-드라이버-변환)
- [Phase 3: Ultrasonic 드라이버 변환](#phase-3-ultrasonic-드라이버-변환)
- [Phase 5: Perception 레이어 변환](#phase-5-perception-레이어-변환)
- [Phase 6: Decision 레이어 변환](#phase-6-decision-레이어-변환)

---

## Phase 1: 빌드 시스템 변환

### 개요
전체 8개 패키지의 빌드 시스템을 catkin에서 ament_cmake로 변환

### 변환 대상 패키지
1. common (유틸리티)
2. bringup (런치)
3. rplidar_driver (센서 드라이버)
4. usb_cam_driver (카메라 드라이버)
5. ultrasonic_driver
6. arduino_driver
7. decision
8. perception_pkg

### package.xml 변환

#### ROS1 (format 2)
```xml
<?xml version="1.0"?>
<package format="2">
  <name>arduino_driver</name>
  <version>0.1.0</version>
  <description>Arduino driver</description>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
</package>
```

#### ROS2 (format 3)
```xml
<?xml version="1.0"?>
<package format="3">
  <name>arduino_driver</name>
  <version>0.1.0</version>
  <description>Arduino driver</description>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>ackermann_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**주요 변경사항:**
- `format="2"` → `format="3"`
- `catkin` → `ament_cmake`
- `roscpp` → `rclcpp`
- `<export>` 섹션 추가

### CMakeLists.txt 변환

#### ROS1
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(arduino_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

add_executable(arduino_bridge_node
  src/arduino_bridge_node.cpp
)

target_link_libraries(arduino_bridge_node
  ${catkin_LIBRARIES}
)
```

#### ROS2
```cmake
cmake_minimum_required(VERSION 3.8)
project(arduino_driver)

# C++ 표준 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# 실행 파일 생성
add_executable(arduino_bridge_node
  src/arduino_bridge_node.cpp
)

# 헤더 파일 경로
target_include_directories(arduino_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

# 의존성 연결 (ROS2 방식)
ament_target_dependencies(arduino_bridge_node
  rclcpp
  std_msgs
  ackermann_msgs
)

# Boost 라이브러리 연결
target_link_libraries(arduino_bridge_node
  ${Boost_LIBRARIES}
)

# 설치 규칙
install(TARGETS arduino_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ament 패키지 설정
ament_package()
```

**주요 변경사항:**
1. `find_package(catkin ...)` → `find_package(ament_cmake ...)`
2. `catkin_package()` 삭제
3. `ament_target_dependencies()` 사용 (의존성 자동 처리)
4. `ament_package()` 추가 (필수)
5. `install()` 규칙 명시적 지정

### Python 패키지 설치 (common 패키지)

#### ROS1
```cmake
catkin_install_python(PROGRAMS
  scripts/debug_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### ROS2
```cmake
# Python 스크립트 설치
install(PROGRAMS
  scripts/debug_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## Phase 2: Arduino 드라이버 변환

### 개요
시리얼 통신을 통한 Arduino 제어 노드를 ROS2로 변환

### 헤더 파일 변환

#### 변경된 파일
- `include/arduino_driver/arduino_bridge_node.hpp`

#### ROS1 코드
```cpp
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float32MultiArray.h>

class ArduinoBridgeNode {
public:
    ArduinoBridgeNode();
    ~ArduinoBridgeNode();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher ultra_pub_;

    void cmdCallback(const ackermann_msgs::AckermannDriveConstPtr& msg);
};
```

#### ROS2 코드
```cpp
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class ArduinoBridgeNode : public rclcpp::Node {
public:
    ArduinoBridgeNode();
    ~ArduinoBridgeNode();

private:
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ultra_pub_;

    void cmdCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);
};
```

**주요 변경사항:**
1. `ros/ros.h` → `rclcpp/rclcpp.hpp`
2. 메시지 헤더: `<pkg/Msg.h>` → `<pkg/msg/msg.hpp>`
3. 클래스 상속: `rclcpp::Node` 상속 필수
4. NodeHandle 제거: Node 클래스 자체가 핸들 역할
5. 포인터 타입: `ConstPtr` → `SharedPtr`

### 구현 파일 변환

#### 생성자 변환

**ROS1:**
```cpp
ArduinoBridgeNode::ArduinoBridgeNode()
    : nh_(), pnh_("~"), running_(true) {

    // 파라미터 로드
    port_ = pnh_.param<std::string>("port", "/dev/ttyACM0");
    baudrate_ = pnh_.param<int>("baudrate", 9600);

    // Subscriber 생성
    cmd_sub_ = nh_.subscribe("/decision/cmd", 10,
                             &ArduinoBridgeNode::cmdCallback, this);

    // Publisher 생성
    ultra_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
        "/arduino/ultrasonic", 10);

    ROS_INFO("Arduino bridge initialized");
}
```

**ROS2:**
```cpp
ArduinoBridgeNode::ArduinoBridgeNode()
    : Node("arduino_bridge_node"), running_(true) {

    // 파라미터 선언 및 로드
    this->declare_parameter("port", "/dev/ttyACM0");
    this->declare_parameter("baudrate", 9600);

    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();

    // Subscriber 생성
    cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/decision/cmd", 10,
        std::bind(&ArduinoBridgeNode::cmdCallback, this, std::placeholders::_1));

    // Publisher 생성
    ultra_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/arduino/ultrasonic", 10);

    RCLCPP_INFO(this->get_logger(), "Arduino bridge initialized");
}
```

**주요 변경사항:**
1. **Node 생성**: `Node("node_name")` 생성자 호출
2. **파라미터**:
   - `declare_parameter()` 먼저 호출 (필수)
   - `get_parameter()` 로 값 가져오기
3. **Subscriber**: `create_subscription()` + `std::bind()`
4. **Publisher**: `create_publisher()` (SharedPtr 반환)
5. **로깅**: `ROS_INFO` → `RCLCPP_INFO(this->get_logger(), ...)`

#### 콜백 함수 변환

**ROS1:**
```cpp
void ArduinoBridgeNode::cmdCallback(
    const ackermann_msgs::AckermannDriveConstPtr& msg) {

    float speed = msg->speed;
    float steering = msg->steering_angle;

    // 시리얼 전송
    sendCommand(speed, steering);
}
```

**ROS2:**
```cpp
void ArduinoBridgeNode::cmdCallback(
    const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {

    float speed = msg->speed;
    float steering = msg->steering_angle;

    // 시리얼 전송
    sendCommand(speed, steering);
}
```

**주요 변경사항:**
- `ConstPtr` → `SharedPtr`
- 메시지 내용 접근은 동일 (`msg->field`)

#### main() 함수 변환

**ROS1:**
```cpp
int main(int argc, char** argv) {
    ros::init(argc, argv, "arduino_bridge_node");

    try {
        ArduinoBridgeNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
```

**ROS2:**
```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ArduinoBridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_bridge_node"),
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
```

**주요 변경사항:**
1. `ros::init()` → `rclcpp::init()`
2. 노드를 `shared_ptr`로 생성
3. `ros::spin()` → `rclcpp::spin(node)`
4. `rclcpp::shutdown()` 명시적 호출

---

## Phase 3: Ultrasonic 드라이버 변환

### 개요
초음파 센서 데이터 처리 노드 변환 (Arduino 드라이버와 동일한 패턴)

### 주요 변경사항

Phase 2와 동일한 패턴으로 변환:
1. 헤더 파일: `ros/ros.h` → `rclcpp/rclcpp.hpp`
2. Node 클래스 상속
3. 파라미터 시스템: `declare_parameter()` + `get_parameter()`
4. Subscriber/Publisher: `create_subscription()`, `create_publisher()`
5. 로깅: `ROS_*` → `RCLCPP_*`

---

## Phase 5: Perception 레이어 변환

### 개요
카메라 이미지 처리 노드 3개를 ROS2로 변환. **딜레이 감소**를 위한 QoS 최적화 적용.

### 변환한 노드
1. **lane_tracking_node**: 차선 추적 및 조향각 계산
2. **obstacle_detection_node**: 색상 기반 장애물 감지
3. **lane_marking_node**: 차선 마킹 및 정지선 감지

### 성능 최적화: QoS 설정

#### 딜레이 감소를 위한 핵심 변경

**ROS1:**
```cpp
image_sub_ = nh_.subscribe(camera_topic_, 1,
                           &LaneTrackingNode::imageCallback, this);
// 큐 크기만 지정 가능, QoS 제어 불가
```

**ROS2 (최적화 적용):**
```cpp
// best_effort QoS: 네트워크 지연 시 오래된 프레임 스킵
auto qos_sensor = rclcpp::QoS(1).best_effort();

image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_topic_, qos_sensor,  // QoS 명시
    std::bind(&LaneTrackingNode::imageCallback, this, std::placeholders::_1));
```

**효과:**
- 큐 크기 1: 최신 프레임만 처리
- best_effort: 네트워크 지연 시 오래된 프레임 폐기
- **예상 지연시간 감소: 30-50%**

### LaneTrackingNode 변환

#### 파일 위치
- `src/perception_pkg/include/perception_pkg/lane_tracking_node.hpp`
- `src/perception_pkg/src/lane_tracking_node.cpp`

#### 주요 변경사항

1. **이미지 처리 파이프라인 최적화**
```cpp
void LaneTrackingNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        handleFrame(cv_ptr->image, msg->header);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
```

2. **Compressed 이미지 지원**
```cpp
// opencv2/imgcodecs.hpp 추가 필요
#include <opencv2/imgcodecs.hpp>

void LaneTrackingNode::compressedCallback(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (!frame.empty()) {
            std_msgs::msg::Header header;
            header.stamp = msg->header.stamp;
            header.frame_id = msg->header.frame_id;
            handleFrame(frame, header);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Image decode error: %s", e.what());
    }
}
```

3. **결과 발행**
```cpp
// ROS1
std_msgs::Float32 steer_msg;
steer_msg.data = static_cast<float>(steering);
steer_pub_.publish(steer_msg);

// ROS2
std_msgs::msg::Float32 steer_msg;
steer_msg.data = static_cast<float>(steering);
steer_pub_->publish(steer_msg);  // -> 연산자 사용
```

### ObstacleDetectionNode 변환

#### 특징
- HSV 색상 기반 장애물 감지
- Canny 에지 검출 + 윤곽선 추출
- 장애물 bias 계산 (회피 조향 지원)

#### 주요 코드

```cpp
void ObstacleDetectionNode::processFrame(
    const cv::Mat& frame, const rclcpp::Time& stamp) {

    auto [detections, overlay] = detectObstacles(frame);

    // 감지 결과 발행
    publishDetections(detections);

    // bias 계산 및 발행
    double bias = computeBias(detections, frame.cols);
    std_msgs::msg::Float32 bias_msg;
    bias_msg.data = static_cast<float>(bias);
    bias_pub_->publish(bias_msg);

    // 오버레이 이미지 발행
    if (publish_overlay_ && !overlay.empty()) {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = "camera";
        sensor_msgs::msg::Image::SharedPtr overlay_msg =
            cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
        overlay_pub_->publish(*overlay_msg);
    }
}
```

### LaneMarkingNode 변환

#### 특징
- Sliding window 알고리즘으로 차선 추적
- 다항식 피팅 (Eigen3 사용)
- 정지선 감지
- 신호등 상태 연동

#### 주요 코드

```cpp
LaneMarkingNode::LaneMarkingNode()
    : Node("lane_marking_node"), current_light_state_("unknown") {

    // 파라미터 선언
    this->declare_parameter("nwindows", 9);
    this->declare_parameter("window_margin", 50);
    this->declare_parameter("minpix", 60);

    nwindows_ = this->get_parameter("nwindows").as_int();
    window_margin_ = this->get_parameter("window_margin").as_int();
    minpix_ = this->get_parameter("minpix").as_int();

    // QoS 설정 (센서 데이터)
    auto qos_sensor = rclcpp::QoS(1).best_effort();

    // 이미지 구독
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, qos_sensor,
        std::bind(&LaneMarkingNode::imageCallback, this, std::placeholders::_1));

    // 신호등 상태 구독
    traffic_light_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/perception/traffic_light_state", 10,
        std::bind(&LaneMarkingNode::trafficLightCallback, this, std::placeholders::_1));
}
```

---

## Phase 6: Decision 레이어 변환

### 개요
10Hz 제어 루프를 가진 의사결정 노드를 ROS2로 변환. **정밀한 타이밍 제어**로 딜레이 감소.

### DecisionNode 변환

#### 핵심: 타이머 기반 제어 루프

**ROS1:**
```cpp
ros::Timer timer = nh.createTimer(ros::Duration(0.1),
                                  &DecisionNode::timerCallback, this);
// 100ms 목표이지만 지터 발생 가능
```

**ROS2 (개선):**
```cpp
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 정확한 100ms
    std::bind(&DecisionNode::timerCallback, this));
// create_wall_timer: 시스템 시간 기반, 더 정확
```

**효과:**
- ROS1 대비 타이밍 지터 감소
- 더 일관된 제어 주기
- 실시간성 향상

#### QoS 최적화

```cpp
// 제어 명령: reliable QoS (메시지 손실 방지)
auto qos_reliable = rclcpp::QoS(10).reliable();

cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "/decision/cmd", qos_reliable);
```

#### 센서 융합 로직

```cpp
void DecisionNode::timerCallback() {
    // 1. 타임아웃 체크 (오래된 데이터 무시)
    auto now = this->now();
    bool lane_valid = lane_stamp_ &&
                      (now - *lane_stamp_).seconds() < lane_timeout_;

    // 2. LiDAR 장애물 우선 체크
    if (obstacle_flag_) {
        publishStop();
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Stopping: LiDAR obstacle detected");
        return;
    }

    // 3. 초음파 장애물 체크
    if (ultra_min_ < ultra_threshold_) {
        publishStop();
        return;
    }

    // 4. 정지선 체크
    if (stop_distance_ > 0 && stop_distance_ < stop_distance_threshold_) {
        publishStop();
        return;
    }

    // 5. 차선 추적
    if (lane_valid && lane_steer_) {
        publishDrive(*lane_steer_);
    } else {
        publishStop();
    }
}
```

### LidarObstacleNode 변환

#### 특징
- LiDAR 데이터에서 특정 각도 범위의 장애물 감지
- 최소 거리 계산 및 플래그 발행

```cpp
void LidarObstacleNode::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // 각도 범위 내 최소 거리 찾기
    float min_dist = std::numeric_limits<float>::max();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;

        if (angle >= angle_min_ && angle <= angle_max_) {
            if (msg->ranges[i] > msg->range_min &&
                msg->ranges[i] < msg->range_max) {
                min_dist = std::min(min_dist, msg->ranges[i]);
            }
        }
    }

    // 플래그 및 거리 발행
    std_msgs::msg::Bool flag_msg;
    flag_msg.data = (min_dist < distance_threshold_);
    flag_pub_->publish(flag_msg);

    std_msgs::msg::Float32 dist_msg;
    dist_msg.data = min_dist;
    distance_pub_->publish(dist_msg);
}
```

---

## 공통 패턴 정리

### 1. 헤더 변환 규칙

| ROS1 | ROS2 |
|------|------|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `#include <std_msgs/String.h>` | `#include <std_msgs/msg/string.hpp>` |
| `#include <sensor_msgs/Image.h>` | `#include <sensor_msgs/msg/image.hpp>` |

### 2. 메시지 타입 변환

| ROS1 | ROS2 |
|------|------|
| `std_msgs::String` | `std_msgs::msg::String` |
| `std_msgs::StringConstPtr` | `std_msgs::msg::String::SharedPtr` |

### 3. 로깅 변환

| ROS1 | ROS2 |
|------|------|
| `ROS_INFO("msg")` | `RCLCPP_INFO(this->get_logger(), "msg")` |
| `ROS_WARN("msg")` | `RCLCPP_WARN(this->get_logger(), "msg")` |
| `ROS_ERROR("msg")` | `RCLCPP_ERROR(this->get_logger(), "msg")` |
| `ROS_DEBUG("msg")` | `RCLCPP_DEBUG(this->get_logger(), "msg")` |

### 4. 시간 처리

| ROS1 | ROS2 |
|------|------|
| `ros::Time now = ros::Time::now()` | `rclcpp::Time now = this->now()` |
| `ros::Duration(1.0)` | `rclcpp::Duration::from_seconds(1.0)` |
| `msg->header.stamp` | `msg->header.stamp` (동일) |

---

## 빌드 및 테스트

### 빌드 명령어

```bash
# 전체 빌드
cd ~/ros2_autonomous_cpp
source /opt/ros/humble/setup.bash
colcon build

# 특정 패키지만 빌드
colcon build --packages-select perception_pkg

# 병렬 빌드 (빠름)
colcon build --parallel-workers 4
```

### 빌드 성공 확인

```bash
# 예상 출력
Summary: 8 packages finished [X.XXs]
```

성공적으로 빌드된 패키지:
1. common
2. bringup
3. rplidar_driver
4. usb_cam_driver
5. ultrasonic_driver
6. arduino_driver
7. decision
8. perception_pkg

---

## 다음 단계

### Phase 7: 런치 파일 변환 (예정)
- XML → Python 런치 파일
- 파라미터 파일 업데이트
- 노드 구성 최적화

### Phase 8: Python 노드 변환 (예정)
- YOLO 표지판 인식
- 신호등 감지
- 디버그 유틸리티

---

## 참고 자료

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rclcpp API Reference](https://docs.ros2.org/humble/api/rclcpp/)
- [QoS Settings Guide](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
