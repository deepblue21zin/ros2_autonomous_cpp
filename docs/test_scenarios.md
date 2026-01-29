# 테스트 시나리오 가이드

테스트 모드를 사용한 모터 및 시스템 테스트 방법을 설명합니다.

---

## 목차

1. [테스트 모드 개요](#테스트-모드-개요)
2. [동작 원리](#동작-원리)
3. [테스트 시나리오](#테스트-시나리오)
4. [실제 테스트 절차](#실제-테스트-절차)
5. [값 범위 정리](#값-범위-정리)
6. [주의사항](#주의사항)

---

## 테스트 모드 개요

### 일반 모드 vs 테스트 모드

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         일반 모드 (test_mode: false)                     │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  센서 입력                    Decision Node                 출력        │
│  ─────────                    ─────────────                 ────        │
│                                                                         │
│  카메라 ──► 차선인식 ──► /lane/steering_angle ──┐                       │
│                                                 │                       │
│  LiDAR ──────────────► /perception/obstacle ────┼──► 체크 ──► 정지?    │
│                                                 │      │               │
│  초음파 ─────────────► /ultrasonic/min_range ───┤      │               │
│                                                 │      ▼               │
│  신호등 ─────────────► /traffic_light_state ────┘   통과 ──► 모터 구동 │
│                                                                         │
│  ⚠️ 센서 데이터 없으면 → 정지 (안전 기능)                               │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│                         테스트 모드 (test_mode: true)                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  센서 입력                    Decision Node                 출력        │
│  ─────────                    ─────────────                 ────        │
│                                                                         │
│  카메라 ──► 차선인식 ──► /lane/steering_angle ──┐                       │
│                                 (있으면 사용)    │                       │
│  LiDAR ─────────────────────────────────────────┼──► 무시               │
│                                                 │      │               │
│  초음파 ────────────────────────────────────────┤      ▼               │
│                                                 │   바로 모터 구동      │
│  신호등 ────────────────────────────────────────┘                       │
│                                                                         │
│  ✅ 센서 데이터 없어도 → 모터 구동 (테스트용)                            │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 동작 원리

### Decision Node 코드 (test_mode 적용)

```cpp
void DecisionNode::timerCallback() {
    // 테스트 모드: 모든 센서 체크 우회
    if (test_mode_) {
        cmd.speed = cruise_speed_;           // 기본 속도 (1.0 m/s)
        cmd.steering_angle = mapSteer(lane_steer_norm_);  // 조향각
        cmd_pub_->publish(cmd);
        return;  // 센서 체크 건너뜀
    }

    // 일반 모드: 센서 체크 수행
    // 1. LiDAR 장애물 체크
    // 2. 초음파 안전 거리 체크
    // 3. 신호등 체크
    // 4. 차선 데이터 타임아웃 체크
    ...
}
```

---

## 테스트 시나리오

### 시나리오 1: 센서 전부 없음 (순수 모터 테스트)

**목적:** Arduino 연결 및 모터 동작 확인

**실행:**
```bash
ros2 launch bringup track_launch.py test_mode:=true
```

**입력:**
| 센서 | 상태 |
|------|------|
| 카메라 | ❌ 없음 |
| LiDAR | ❌ 없음 |
| 초음파 | ❌ 없음 |

**동작 흐름:**
```
lane_steer_norm_ = 0.0 (초기값, 데이터 없으면 0 유지)
        │
        ▼
cmd.speed = cruise_speed_ (1.0 m/s 기본값)
cmd.steering_angle = mapSteer(0.0) = 0.0 (직진)
        │
        ▼
Arduino로 전송: "V:127,S:90" (중간 속도, 서보 중앙)
        │
        ▼
🚗 차량: 일정 속도로 직진
```

**예상 결과:**
- 모터가 일정 속도로 회전
- 서보가 중앙 위치 유지
- 차량이 직진

---

### 시나리오 2: 카메라만 연결 (차선 추종 테스트)

**목적:** 차선 인식 및 조향 동작 확인

**실행:**
```bash
ros2 launch bringup track_launch.py test_mode:=true
```

**입력:**
| 센서 | 상태 |
|------|------|
| 카메라 | ✅ 연결됨 → 차선 인식 |
| LiDAR | ❌ 없음 |
| 초음파 | ❌ 없음 |

**동작 흐름:**
```
차선 인식 결과: lane_steer_norm_ = 0.3 (우측으로 틀어야 함)
        │
        ▼
cmd.speed = 1.0 m/s
cmd.steering_angle = mapSteer(0.3) = 0.18 rad (약 10도)
        │
        ▼
Arduino로 전송: "V:127,S:100" (속도 중간, 서보 우측)
        │
        ▼
🚗 차량: 일정 속도로 우회전하며 차선 따라감
```

**예상 결과:**
- 차선 방향에 따라 서보 각도 변화
- `/lane/steering_angle` 토픽 발행 확인

---

### 시나리오 3: 수동 조향 테스트 (topic pub)

**목적:** 특정 조향각에서 모터/서보 동작 확인

**터미널 1 - 시스템 실행:**
```bash
ros2 launch bringup track_launch.py test_mode:=true
```

**터미널 2 - 수동 명령:**
```bash
# 좌회전 (-0.5)
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: -0.5}"

# 우회전 (0.5)
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: 0.5}"

# 강한 좌회전 (-1.0)
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: -1.0}"

# 강한 우회전 (1.0)
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: 1.0}"

# 직진 (0.0)
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: 0.0}"
```

**동작 흐름 (0.5 입력 시):**
```
수동 입력: lane_steer_norm_ = 0.5
        │
        ▼
cmd.speed = 1.0 m/s
cmd.steering_angle = mapSteer(0.5) = 0.3 rad (약 17도)
        │
        ▼
Arduino로 전송: "V:127,S:107"
        │
        ▼
🚗 차량: 우회전
```

---

### 시나리오 4: 속도 조절 테스트

**목적:** 다양한 속도에서 모터 동작 확인

**설정 변경 (arduino.yaml):**
```yaml
# 테스트용 낮은 속도
cruise_speed_mps: 0.5  # 기본 1.0에서 0.5로 변경
```

또는 런타임에 파라미터 변경:
```bash
ros2 param set /decision_node cruise_speed_mps 0.5
```

---

## 실제 테스트 절차

### 1단계: 바퀴 들어올린 상태로 테스트

```bash
# 컨테이너 접속
docker exec -it adas_container bash

# 빌드
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 테스트 모드 실행
ros2 launch bringup track_launch.py test_mode:=true
```

**예상 로그:**
```
[decision_node]: DecisionNode initialized (10Hz)
[decision_node]:   cruise_speed: 1.00 m/s
[decision_node]:   test_mode: true
[decision_node]: ⚠️ TEST MODE ENABLED - Sensors bypassed!
```

### 2단계: 모터 명령 모니터링

```bash
# 다른 터미널에서
docker exec -it adas_container bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# 모터 명령 모니터링
ros2 topic echo /arduino/cmd
```

**예상 출력:**
```yaml
speed: 1.0
steering_angle: 0.0
---
speed: 1.0
steering_angle: 0.0
---
```

### 3단계: 수동 조향 테스트

```bash
# 좌회전
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: -0.5}"

# 우회전
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: 0.5}"

# 직진
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: 0.0}"
```

### 4단계: 긴급 정지

```bash
# 방법 1: Ctrl+C로 노드 종료

# 방법 2: 시리얼 직접 명령
echo "S" > /dev/ttyACM0

# 방법 3: 정지 명령 발행
ros2 topic pub --once /lane/steering_angle std_msgs/msg/Float32 "{data: 0.0}"
```

---

## 값 범위 정리

### 입력값 (조향)

| 변수 | 범위 | 설명 |
|------|------|------|
| `lane_steer_norm_` | -1.0 ~ 1.0 | 차선 조향 (좌: -, 우: +) |
| `-1.0` | 최대 좌회전 | |
| `0.0` | 직진 | |
| `1.0` | 최대 우회전 | |

### 출력값 (Arduino)

| 변수 | 범위 | 설명 |
|------|------|------|
| `steering_angle` | -0.6 ~ 0.6 rad | 실제 조향각 (약 ±34도) |
| PWM | 0 ~ 255 | 모터 속도 (127 = 중간) |
| Servo | 60 ~ 120 | 서보 각도 (90 = 중앙) |

### 조향각 매핑

| 입력 (norm) | 출력 (rad) | 출력 (도) | 서보 각도 |
|-------------|------------|-----------|-----------|
| -1.0 | -0.6 | -34° | 60 |
| -0.5 | -0.3 | -17° | 75 |
| 0.0 | 0.0 | 0° | 90 |
| 0.5 | 0.3 | 17° | 105 |
| 1.0 | 0.6 | 34° | 120 |

---

## 주의사항

### 안전 수칙

1. **차량을 들어올린 상태에서 먼저 테스트**
   - 바퀴가 땅에 닿지 않게
   - 예상치 못한 움직임 방지

2. **속도를 낮게 설정**
   ```yaml
   # arduino.yaml
   max_speed_mps: 0.5  # 테스트용 낮은 속도
   ```

3. **긴급 정지 준비**
   ```bash
   echo "S" > /dev/ttyACM0
   ```

4. **테스트 후 반드시 test_mode 비활성화**
   ```bash
   ros2 launch bringup track_launch.py  # test_mode 없이 실행
   ```

### 체크리스트

- [ ] Arduino 연결 확인 (`ls /dev/ttyACM*`)
- [ ] 차량 들어올림 확인
- [ ] 긴급 정지 명령 준비
- [ ] 속도 설정 확인 (낮게)
- [ ] 테스트 모드 활성화 확인 (로그)
- [ ] 모터 동작 확인
- [ ] 서보 동작 확인
- [ ] 테스트 완료 후 test_mode 비활성화

---

## 문제 해결

### 모터가 동작하지 않을 때

1. Arduino 연결 확인
   ```bash
   ls /dev/ttyACM*
   ```

2. 시리얼 권한 확인
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

3. Arduino 노드 로그 확인
   ```bash
   ros2 topic echo /arduino/cmd
   ```

### 서보가 중앙에서 벗어나 있을 때

```yaml
# arduino.yaml
center_servo_deg: 90.0  # 중앙값 조정
```

### 조향 방향이 반대일 때

차선 인식 노드의 조향 부호 확인 또는:
```yaml
# lane_params.yaml
steer_invert: true  # 조향 반전 (있는 경우)
```
