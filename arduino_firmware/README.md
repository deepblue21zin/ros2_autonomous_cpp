# Arduino Ackermann Drive Firmware

ROS2 Humble과 통신하는 Ackermann Drive 제어 펌웨어입니다.

## 하드웨어 요구사항

### 필수
- Arduino (Uno/Nano/Mega 등)
- 조향 서보 모터
- ESC + 브러시리스 모터 **또는** DC 모터 드라이버 + DC 모터

### 선택
- 초음파 센서 (HC-SR04)

## 핀 연결

```
Arduino Pin | 장치
----------- | -------------
9           | 조향 서보 PWM
10          | ESC/모터 PWM
7           | 초음파 TRIG (선택)
8           | 초음파 ECHO (선택)
GND         | 공통 그라운드
```

## 펌웨어 설정

### 1. RC ESC 사용 시 (기본)

```cpp
const bool USE_RC_ESC = true;
const int ESC_NEUTRAL = 90;
const int ESC_FORWARD_MIN = 91;
const int ESC_FORWARD_MAX = 180;
```

### 2. DC 모터 드라이버 사용 시 (L298N 등)

```cpp
const bool USE_RC_ESC = false;
// PWM 0~255로 속도 제어
```

### 3. 조향 서보 캘리브레이션

```cpp
const int CENTER_SERVO_DEG = 90;      // 직진 각도 조정
const int MAX_STEER_DEG = 30;         // 최대 회전 각도
```

## 업로드 방법

1. **Arduino IDE 열기**

2. **펌웨어 열기**
   ```
   arduino_firmware/ackermann_drive_firmware.ino
   ```

3. **보드 선택**
   - Tools → Board → Arduino Uno (또는 사용 중인 보드)

4. **포트 선택**
   - Tools → Port → /dev/ttyACM0 (또는 자동 감지된 포트)

5. **업로드**
   - Upload 버튼 클릭

## ROS2 파라미터 매핑

### arduino.yaml 설정

```yaml
port: "/dev/ttyACM0"
baudrate: 115200
use_legacy_cmd: false          # 연속 모드 사용
center_servo_deg: 90
max_steer_deg: 30
max_speed_mps: 1.0
```

### ROS2 → Arduino 변환

#### 조향각 변환

ROS2에서 보내는 `steering_angle` (라디안):
```
-0.52 rad (좌회전 30°)  → Arduino: 60°
 0.00 rad (직진)        → Arduino: 90°
+0.52 rad (우회전 30°)  → Arduino: 120°
```

변환 공식 (arduino_bridge_node.cpp:145-149):
```cpp
double steer_deg = steering_angle_rad * 180 / PI;
steer_deg = clamp(steer_deg, -max_steer_deg, max_steer_deg);
servo_deg = center_servo_deg + steer_deg;
```

#### 속도 변환

ROS2에서 보내는 `speed` (m/s):
```
1.0 m/s   → Arduino: 255 (PWM, 최대)
0.5 m/s   → Arduino: 128 (PWM, 중간)
0.0 m/s   → Arduino: 0 (정지)
```

변환 공식 (arduino_bridge_node.cpp:139-143):
```cpp
double ratio = abs(speed_mps) / max_speed_mps;
pwm = (int)(ratio * 255);  // 0~255, 항상 양수
```

**⚠️ 주의: 현재 ROS2 코드는 후진을 지원하지 않습니다!**
- `speedToPwm()`이 `abs(speed)`를 사용하여 항상 양수 반환
- 후진 기능이 필요하면 ROS2 코드 수정 필요

## 시리얼 통신 형식

### ROS2 → Arduino (명령)

```
V:128,S:90\n   (중속 전진, 직진)
V:255,S:75\n   (최고속 전진, 좌회전)
V:0,S:90\n     (정지, 직진)
```

- `V:` 속도 PWM (0 ~ 255, 항상 양수)
  - 0 = 정지
  - 255 = 최대 속도
- `S:` 조향각 (60 ~ 120도)
  - 60 = 최대 좌회전
  - 90 = 직진
  - 120 = 최대 우회전
- `\n` 줄바꿈 (명령 종료)

**실제 ROS2에서 전송되는 예시:**
```bash
# ros2 topic echo /arduino/cmd 명령 결과
speed: 1.0              # max_speed_mps = 1.0
steering_angle: -0.071  # 약 -4도

# Arduino로 전송되는 명령:
V:255,S:86
# PWM=255 (최고속), Servo=86도 (약간 좌회전)
```

### Arduino → ROS2 (선택, 센서 데이터)

```
ULTRASONIC:25.5\n
```

## 테스트 방법

### 1. 시리얼 모니터 테스트

Arduino IDE에서:
1. Tools → Serial Monitor
2. Baud rate: 115200
3. Line ending: Newline

명령 입력:
```
V:128,S:90    → 중속 전진, 직진
V:0,S:90      → 정지, 직진
V:100,S:70    → 전진, 좌회전 (20도)
V:100,S:110   → 전진, 우회전 (20도)
V:255,S:60    → 최고속, 최대 좌회전
V:255,S:120   → 최고속, 최대 우회전
```

**참고:** 후진은 현재 ROS2 코드에서 지원하지 않습니다.

### 2. ROS2 통신 테스트

```bash
# Arduino 연결 확인
ls -l /dev/ttyACM0

# 시리얼 데이터 확인
sudo cat /dev/ttyACM0

# ROS2 런치 (다른 터미널)
ros2 launch bringup track_launch.py test_mode:=true

# 명령 토픽 확인
ros2 topic echo /arduino/cmd
```

## 안전 기능

### 1. 명령 타임아웃
- 500ms 동안 명령이 없으면 자동으로 정지
- 통신 끊김 시 안전

### 2. 범위 제한
- 속도: -255 ~ 255
- 조향: center ± max_steer_deg

### 3. 긴급 정지
- `emergencyStop()` 함수로 즉시 정지

## 문제 해결

### 모터가 작동하지 않음

1. **ESC 전원 확인**
   - 배터리 연결 확인
   - ESC LED 상태 확인

2. **ESC 캘리브레이션**
   ```cpp
   // setup()에 추가
   esc.write(180); // 최대
   delay(2000);
   esc.write(0);   // 최소
   delay(2000);
   esc.write(90);  // 중립
   ```

3. **PWM 값 확인**
   - 시리얼 모니터에서 디버그 출력 활성화
   ```cpp
   Serial.print("Speed: ");
   Serial.println(currentSpeed);
   ```

### 서보가 떨림

1. **중앙값 조정**
   ```cpp
   const int CENTER_SERVO_DEG = 88; // 실제 직진 각도
   ```

2. **전원 분리**
   - 서보와 Arduino 전원 분리 (공통 GND만)

### 시리얼 통신 안됨

1. **포트 확인**
   ```bash
   ls /dev/ttyACM*
   dmesg | tail
   ```

2. **권한 확인**
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

## 코드 수정 가이드

### 다른 모터 드라이버 사용

예: L298N
```cpp
#define MOTOR_PWM_PIN 10
#define MOTOR_DIR_PIN 12

void setMotorSpeed(int speed) {
  int pwmValue = abs(speed);
  bool forward = (speed > 0);

  analogWrite(MOTOR_PWM_PIN, pwmValue);
  digitalWrite(MOTOR_DIR_PIN, forward ? HIGH : LOW);
}
```

### 추가 센서 연결

```cpp
// loop()에 추가
readIMU();
readEncoder();
sendSensorData();
```

## 라이선스

MIT License
