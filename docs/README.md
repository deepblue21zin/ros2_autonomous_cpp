# ROS1 Noetic → ROS2 Humble 변환 문서

이 문서는 당시 마이그레이션 작업의 요약본입니다. 현재 활성 운영 문서는 `docs/`에 남겨두고, 상세 변경 이력과 트러블슈팅 기록은 `archive/docs_archive/`로 이동했습니다.

## 프로젝트 개요

이 문서는 자율주행 차량 시스템을 ROS1 Noetic에서 ROS2 Humble로 변환한 전체 과정을 기록합니다.

### 변환 목표

1. **주요 목표: 시스템 딜레이 감소**
   - ROS1 Noetic의 높은 지연시간 문제 해결
   - DDS 미들웨어를 통한 통신 오버헤드 감소
   - QoS 정책을 활용한 실시간 성능 개선

2. **부가 목표**
   - ROS2의 최신 기능 활용
   - 더 나은 멀티스레딩 지원
   - 개선된 파라미터 시스템

### 시스템 아키텍처

```
┌─────────────────┐
│   Sensors       │
│  - RPLiDAR     │
│  - Camera      │
│  - Ultrasonic  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Perception     │
│  - Lane Track   │
│  - Obstacles    │
│  - Traffic Sign │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│   Decision      │
│  - 10Hz Loop   │
│  - Fusion      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│   Actuators     │
│  - Arduino      │
│  - Steering     │
└─────────────────┘
```

## 변환 현황

### ✅ 완료된 작업

| Phase | 패키지/모듈 | 상태 | 설명 |
|-------|------------|------|------|
| Phase 1 | 빌드 시스템 (8개 패키지) | ✅ | catkin → ament_cmake |
| Phase 2 | arduino_driver | ✅ | 시리얼 통신 드라이버 |
| Phase 3 | ultrasonic_driver | ✅ | 초음파 센서 처리 |
| Phase 5 | perception_pkg (C++) | ✅ | 3개 인식 노드 변환 |
| Phase 6 | decision | ✅ | 제어 로직 (10Hz) |
| Phase 7 | 런치 시스템 | ✅ | XML → Python 런치 (5개) |
| Phase 8 | Python 노드 | 🔄 | 2개 변환 완료, 16개 남음 |

**총 8개 패키지 빌드 성공:**
- arduino_driver
- ultrasonic_driver
- decision (decision_node, lidar_obstacle_node)
- perception_pkg (lane_tracking_node, obstacle_detection_node, lane_marking_node)
- rplidar_driver (wrapper)
- usb_cam_driver (wrapper)
- common (utilities)
- bringup (launch)

**런치 파일 (5개 변환 완료):**
- rplidar_launch.py
- usb_cam_launch.py
- lane_bringup_launch.py
- track_launch.py
- mission_launch.py

**Python 노드 (2개 변환 완료):**
- lidar_obstacle_node.py
- decision_node_2026.py

### 🚧 진행 중인 작업

| Phase | 작업 | 상태 |
|-------|------|------|
| Phase 8 | Python 노드 변환 | 16개 남음 (YOLO 노드 포함) |

## 주요 성능 개선

### 1. 통신 지연시간 감소

**QoS 프로필 최적화:**
```cpp
// 센서 데이터: best_effort (오래된 데이터 스킵)
auto qos_sensor = rclcpp::QoS(1).best_effort();

// 제어 명령: reliable (메시지 손실 방지)
auto qos_reliable = rclcpp::QoS(10).reliable();
```

### 2. 제어 루프 정밀도 향상

**ROS1:**
```cpp
ros::Timer timer = nh.createTimer(ros::Duration(0.1), callback);
// 100ms 목표, 실제 지터 많음
```

**ROS2:**
```cpp
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 정확한 100ms
    std::bind(&DecisionNode::timerCallback, this));
```

### 3. 이미지 처리 최적화

- **큐 크기 1**: 최신 프레임만 처리
- **best_effort QoS**: 네트워크 지연 시 프레임 스킵
- **예상 성능 향상**: 30-50% 지연시간 감소

## 문서 구성

- [INSTALLATION.md](INSTALLATION.md) - 설치한 ROS2 패키지 목록
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - C++ 노드 상세 변환 가이드
- [../archive/docs_archive/PYTHON_MIGRATION.md](../archive/docs_archive/PYTHON_MIGRATION.md) - Python 노드 변환 가이드
- [../archive/docs_archive/API_CHANGES.md](../archive/docs_archive/API_CHANGES.md) - ROS1 → ROS2 API 변경사항
- [PERFORMANCE.md](PERFORMANCE.md) - 성능 최적화 세부사항
- [../archive/docs_archive/BUILD_SYSTEM.md](../archive/docs_archive/BUILD_SYSTEM.md) - 빌드 시스템 변경사항
- [../archive/docs_archive/CHANGELOG.md](../archive/docs_archive/CHANGELOG.md) - 상세 변경 이력

## 빠른 시작

### 환경 설정
```bash
cd ~/ros2_autonomous_cpp
source /opt/ros/humble/setup.bash
```

### 빌드
```bash
colcon build
source install/setup.bash
```

### 실행

**트랙 주행 모드:**
```bash
ros2 launch bringup track_launch.py
```

**미션 모드 (전체 시스템):**
```bash
ros2 launch bringup mission_launch.py
```

**옵션 사용:**
```bash
# C++ 노드 사용 (기본값)
ros2 launch bringup mission_launch.py use_cpp:=true

# Python 노드 사용 (변환 완료된 노드만)
ros2 launch bringup mission_launch.py use_cpp:=false

# AI 모드
ros2 launch bringup mission_launch.py decision_mode:=ai
```

**개별 센서 실행:**
```bash
# RPLiDAR
ros2 launch rplidar_driver rplidar_launch.py

# USB 카메라
ros2 launch usb_cam_driver usb_cam_launch.py

# 차선 인식 (perception만)
ros2 launch perception_pkg lane_bringup_launch.py
```

## 다음 단계

1. **Phase 8 완료: 나머지 Python 노드 변환 (16개)**
   - 우선순위 1: decision 노드들 (decision_node.py, decision_node_ai.py)
   - 우선순위 2: 드라이버 노드들 (arduino_bridge_node.py, ultrasonic_processor_node.py)
   - 우선순위 3: YOLO 기반 노드들 (speed_sign_node.py, traffic_light_node.py)
   - 선택사항: 디버그/모니터링 노드들

2. **통합 테스트**
   - 실제 차량 테스트
   - 지연시간 측정 및 검증
   - 성능 벤치마크
   - 안정성 테스트

3. **최적화 및 튜닝**
   - QoS 프로필 미세 조정
   - 파라미터 최적화
   - 성능 프로파일링

## 참고 자료

- [ROS2 Migration Guide (Official)](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)
- [ROS2 Design](https://design.ros2.org/)
- [DDS and ROS2](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)
