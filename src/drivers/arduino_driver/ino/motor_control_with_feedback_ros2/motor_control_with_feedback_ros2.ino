#include <Arduino.h>

// ==================== ROS2 호환 펌웨어 ====================
// 프로토콜: V:PWM,S:SERVO (예: V:100,S:90)
// 기존 단일 문자 명령도 유지 (F, B, L, R, S)
// =======================================================

// ==================== 핀 설정 ====================
// 차동 구동 방식: 좌/우 바퀴 독립 제어 + 서보 조향
// 모터 드라이버 1: 좌측 바퀴
int motorLeft_IN1 = 2;    // 모터 드라이버 1 IN1 (좌)
int motorLeft_IN2 = 3;    // 모터 드라이버 1 IN2 (파)

// 모터 드라이버 2: 우측 바퀴
int motorRight_IN1 = 4;   // 모터 드라이버 2 IN1 (쥬)
int motorRight_IN2 = 5;   // 모터 드라이버 2 IN2 (노)

// 모터 드라이버 3: 서보 모터 (조향)
int servo_IN1 = 6;        // 모터 드라이버 3 IN1 (갈)
int servo_IN2 = 7;        // 모터 드라이버 3 IN2 (빨)

// ==================== 조향각 피드백 설정 (NEW) ====================
// 가변저항 핀 (조향 피드백용)
int STEERING_POT_PIN = A0;        // 가변저항 아날로그 입력 핀

// 가변저항 캘리브레이션 값 (INTERNAL2V56 기준, K 명령으로 재측정 필요)
// ADC = (전압 / 2.56) * 1023
// 실측 ADC 기준: 좌 909, 우 589 (전압 환산 약 2.27V, 1.47V)
int POT_LEFT = 890;               // 최대 좌회전 시 ADC 값
int POT_RIGHT = 640;              // 최대 우회전 시 ADC 값
int POT_CENTER = (POT_RIGHT + POT_LEFT)/2; 

// 조향각 범위 (도)
float ANGLE_LEFT = 120.0;          // 최대 좌회전 각도
float ANGLE_CENTER_DEG = 90.0;    // 중앙 각도
float ANGLE_RIGHT = 60.0;        // 최대 우회전 각도

// 피드백 필터링
int pot_readings[5];              // 이동평균 필터 버퍼
int pot_index = 0;
bool pot_buffer_filled = false;

// 피드백 데이터 전송 주기 설정
bool ENABLE_STEERING_FEEDBACK = true;    // 피드백 데이터 시리얼 전송 여부 (디버그용)
int FEEDBACK_SEND_INTERVAL = 10;        // N번 루프마다 피드백 전송 (5 = 센서데이터와 동일 주기)

// ==================== PID 조향 제어 설정 ====================
// PID 게인 (시리얼 'T' 명령으로 실시간 튜닝 가능)
float PID_KP = 10.0;              // 비례 게인
float PID_KI = 0.0;              // 적분 게인
float PID_KD = 2.0;              // 미분 게인

// PID 제한
float PID_DEADBAND = 2.5;        // 오차 허용 범위 (도) - 목표 근처에서 모터 정지
int PID_MIN_PWM = 60;            // 모터 구동 최소 PWM (정지 마찰 극복)
int PID_MAX_PWM = 255;           // 모터 구동 최대 PWM (안전 제한)
float PID_INTEGRAL_LIMIT = 50.0; // 적분 와인드업 방지

// PID 상태
float pid_integral = 0.0;
float pid_prev_error = 0.0;
unsigned long pid_last_time = 0;
bool pid_enabled = true;          // PID 활성/비활성 전환

// 초음파 센서 6개
int trigPins[6] = {22, 24, 26, 28, 30, 32};  // Trig 핀 배열
int echoPins[6] = {23, 25, 27, 29, 31, 33};  // Echo 핀 배열

// 센서 위치 인덱스
#define FRONT       0   // 전방
#define FRONT_LEFT  1   // 좌전방
#define FRONT_RIGHT 2   // 우전방
#define REAR        3   // 후방
#define REAR_LEFT   4   // 좌후방
#define REAR_RIGHT  5   // 우후방

// ==================== 속도 및 각도 설정 ====================
int SPEED_FORWARD = 150;      // 전진 속도 (0~255)
int SPEED_BACKWARD = 120;     // 후진 속도
int SPEED_TURN = 100;         // 회전 속도


// 서보 조향 각도 (새 매핑: 좌회전=120°, 우회전=60°)
int SERVO_CENTER = 90;        // 중앙 (직진)
int SERVO_LEFT_MAX = 120;     // 최대 좌회전 각도
int SERVO_RIGHT_MAX = 69;     // 최대 우회전 각도
int SERVO_LEFT_SOFT = (SERVO_CENTER + SERVO_LEFT_MAX)/2;    // 약한 좌회전
int SERVO_RIGHT_SOFT = (SERVO_CENTER + SERVO_RIGHT_MAX)/2;    // 약한 우회전


// ==================== 전역 변수 ====================
char command = 'S';               // 수신한 명령
long distances[6];                // 6개 초음파 센서 거리값 (cm)
int target_steering_angle = 90;   // 목표 조향 각도 (명령으로 설정된 값)
float actual_steering_angle = 90.0;  // 실제 조향 각도 (가변저항에서 읽은 값)
int loop_count = 0;               // 루프 카운터 (초음파 측정 빈도 조절용)
int feedback_count = 0;           // 피드백 전송 카운터

// ROS2 프로토콜용 버퍼
String serial_buffer = "";        // 시리얼 명령 버퍼
int current_pwm = 0;              // 현재 PWM 값

// ==================== 초기화 ====================
void setup() {
  Serial.begin(115200);

  // 좌측 바퀴 모터 핀 설정
  pinMode(motorLeft_IN1, OUTPUT);
  pinMode(motorLeft_IN2, OUTPUT);

  // 우측 바퀴 모터 핀 설정
  pinMode(motorRight_IN1, OUTPUT);
  pinMode(motorRight_IN2, OUTPUT);

  // 서보모터 핀 설정 (모터 드라이버로 제어)
  pinMode(servo_IN1, OUTPUT);
  pinMode(servo_IN2, OUTPUT);

  // 가변저항 핀 설정 (아날로그 입력)
  pinMode(STEERING_POT_PIN, INPUT);

  // ADC 기준 전압을 2.56V로 변경 (분해능 향상: 81→160 counts)
  // 주의: 모든 아날로그 입력이 2.56V 기준으로 변환됨
  analogReference(INTERNAL2V56);

  // 초음파 센서 6개 핀 설정
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // 이동평균 필터 버퍼 초기화 (중앙값으로)
  for (int i = 0; i < 5; i++) {
    pot_readings[i] = POT_CENTER;  // 357 (1.75V)
  }

  // PID 타이머 초기화
  pid_last_time = millis();

  // 초기 상태: 정지, 직진
  motor_stop();
  steering_center();

  Serial.println("Arduino Ready!");
  Serial.println("ROS2 Compatible Firmware (PID Steering)");
  Serial.println("Protocol: V:PWM,S:SERVO or single char (F,B,L,R,S)");
  Serial.println("PID Tuning: T:P:5.0, T:I:0.5, T:D:1.0, T:B:2.0, T:ON, T:OFF");
  Serial.println("Example: V:100,S:90");
}

// ==================== 메인 루프 (최적화) ====================
void loop() {
  // 1. 명령 수신 (라인 기반 파싱)
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      // 줄바꿈 시 명령 처리
      if (serial_buffer.length() > 0) {
        processCommand(serial_buffer);
        serial_buffer = "";
      }
    } else {
      serial_buffer += c;

      // 단일 문자 명령 즉시 처리 (F, B, L, R, S, K, D)
      if (serial_buffer.length() == 1 &&
          (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S' ||
           c == 'l' || c == 'r' || c == 'K' || c == 'D')) {
        processCommand(serial_buffer);
        serial_buffer = "";
      }
    }
  }

  // 2. PID 조향 제어 (매 루프 실행 - 가변저항 피드백 기반)
  if (pid_enabled) {
    pid_steering_control();
  } else if (ENABLE_STEERING_FEEDBACK) {
    // PID 비활성 시에도 피드백 읽기 (모니터링용)
    read_steering_feedback();
  }

  // 3. 초음파 측정은 5회 중 1회만 (딜레이 감소)
  loop_count++;
  if (loop_count >= 5) {
    //measure_all_ultrasonic();
    //send_sensor_data();
    loop_count = 0;
  }

  // 4. 조향 피드백 전송 (별도 주기)
  feedback_count++;
  if (ENABLE_STEERING_FEEDBACK && feedback_count >= FEEDBACK_SEND_INTERVAL) {
    send_steering_feedback();
    feedback_count = 0;
  }

  delay(10);  // 10ms 루프
}

// ==================== 명령 처리 (ROS2 프로토콜 + 단일 문자) ====================
void processCommand(String cmd) {
  cmd.trim();

  // ROS2 프로토콜: V:PWM,S:SERVO
  if (cmd.startsWith("V:") && cmd.indexOf(",S:") > 0) {
    parseROS2Command(cmd);
  }
  // PID 튜닝 명령: T:P:5.0, T:I:0.5, T:D:1.0, T:B:2.0, T:ON, T:OFF
  else if (cmd.startsWith("T:")) {
    parsePIDTuning(cmd);
  }
  // 단일 문자 명령
  else if (cmd.length() == 1) {
    executeCommand(cmd.charAt(0));
  }
}

// ROS2 프로토콜 파싱: V:PWM,S:SERVO
void parseROS2Command(String cmd) {
  // 예: "V:100,S:90"
  int vIndex = cmd.indexOf("V:");
  int sIndex = cmd.indexOf(",S:");

  if (vIndex >= 0 && sIndex > 0) {
    String pwmStr = cmd.substring(vIndex + 2, sIndex);
    String servoStr = cmd.substring(sIndex + 3);

    int pwm = pwmStr.toInt();
    int servo = servoStr.toInt();

    // 범위 제한
    pwm = constrain(pwm, 0, 255);
    int servo_min = min(SERVO_LEFT_MAX, SERVO_RIGHT_MAX);
    int servo_max = max(SERVO_LEFT_MAX, SERVO_RIGHT_MAX);
    servo = constrain(servo, servo_min, servo_max);

    // 명령 실행
    current_pwm = pwm;
    command = 'V';

    if (pwm > 0) {
      motor_forward(pwm);
    } else {
      motor_stop();
    }

    setServoAngle(servo);
  }
}

// ==================== 조향각 피드백 함수 (NEW) ====================

// 가변저항에서 실제 조향각 읽기
void read_steering_feedback() {
  // 아날로그 값 읽기 (0~1023)
  int raw_value = analogRead(STEERING_POT_PIN);

  // 이동평균 필터 적용 (노이즈 제거)
  pot_readings[pot_index] = raw_value;
  pot_index = (pot_index + 1) % 5;
  if (pot_index == 0) pot_buffer_filled = true;

  // 평균 계산
  int sum = 0;
  int count = pot_buffer_filled ? 5 : (pot_index + 1);
  for (int i = 0; i < count; i++) {
    sum += pot_readings[i];
  }
  int filtered_value = sum / count;

  // ADC 값을 각도로 변환
  actual_steering_angle = map_pot_to_angle(filtered_value);
}

// 가변저항 ADC 값을 각도로 변환

// ADC 높을수록 좌회전(120°), 낮을수록 우회전(60°)
// POT_LEFT=909 (좌회전), POT_CENTER=749, POT_RIGHT=589 (우회전)
float map_pot_to_angle(int pot_value) {
  // 범위 제한 (POT_RIGHT=589 < POT_LEFT=909)
  pot_value = constrain(pot_value, POT_RIGHT, POT_LEFT);

  // 선형 보간 (구간별 매핑)
  float angle;
  if (pot_value >= POT_CENTER) {
    // 좌회전 영역: POT_CENTER(749) ~ POT_LEFT(909) → 90° ~ 120°
    angle = map_float(pot_value, POT_CENTER, POT_LEFT, ANGLE_CENTER_DEG, ANGLE_LEFT);
  } else {
    // 우회전 영역: POT_RIGHT(589) ~ POT_CENTER(749) → 60° ~ 90°
    angle = map_float(pot_value, POT_RIGHT, POT_CENTER, ANGLE_RIGHT, ANGLE_CENTER_DEG);

  }

  return angle;
}

// float용 map 함수
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 조향 피드백 데이터 전송 (디버그용)
void send_steering_feedback() {
  // 형식: "STR:target,actual,error,adc,voltage"
  // target: 목표 조향각 (명령값)
  // actual: 실제 조향각 (가변저항 측정값)
  // error: 오차 (target - actual)
  // adc: 가변저항 원시 ADC 값
  // voltage: 가변저항 전압값

  int raw_adc = analogRead(STEERING_POT_PIN);
  float voltage = (raw_adc / 1023.0) * 2.56;
  float error = (float)target_steering_angle - actual_steering_angle;

  Serial.print("STR:");
  Serial.print(target_steering_angle);
  Serial.print(",");
  Serial.print(actual_steering_angle, 1);  // 소수점 1자리
  Serial.print(",");
  Serial.print(error, 1);
  Serial.print(",");
  Serial.print(raw_adc);
  Serial.print(",");
  Serial.print(voltage, 2);  // 소수점 2자리
  Serial.println();
}

// ==================== PID 조향 제어 함수 ====================

// PID 기반 조향 제어 (매 루프마다 실행)
void pid_steering_control() {
  // 1. 가변저항에서 실제 조향각 읽기
  read_steering_feedback();

  // 2. 오차 계산
  float error = (float)target_steering_angle - actual_steering_angle;

  // 3. 데드밴드: 목표 근처에서 모터 정지 + 적분 리셋
  if (abs(error) < PID_DEADBAND) {
    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, 0);
    pid_integral = 0;
    pid_prev_error = error;
    return;
  }

  // 4. 시간 차분 계산
  unsigned long now = millis();
  float dt = (now - pid_last_time) / 1000.0;  // 초 단위
  if (dt <= 0 || dt > 1.0) dt = 0.01;  // 비정상 값 방지

  // 5. 적분 항 (와인드업 방지)
  pid_integral += error * dt;
  pid_integral = constrain(pid_integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

  // 6. 미분 항
  float derivative = (error - pid_prev_error) / dt;

  // 7. PID 출력 계산
  float output = PID_KP * error + PID_KI * pid_integral + PID_KD * derivative;

  // 8. 상태 업데이트
  pid_prev_error = error;
  pid_last_time = now;

  // 9. PWM 출력 (방향 + 크기)
  // 새 매핑: ANGLE_LEFT=120°(ADC 높음), ANGLE_RIGHT=60°(ADC 낮음)
  int pwm = constrain(abs((int)output), PID_MIN_PWM, PID_MAX_PWM);

  if (output > 0) {

    // target > actual → 좌회전 방향 (angle 증가, ADC 높아져야 함)

    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, pwm);
  } else {
    // target < actual → 우회전 방향 (angle 감소, ADC 낮아져야 함)
    analogWrite(servo_IN1, pwm);
    analogWrite(servo_IN2, 0);
  }
}

// PID 튜닝 명령 처리 (시리얼)
// 형식: T:P:5.0, T:I:0.5, T:D:1.0, T:B:2.0, T:ON, T:OFF
void parsePIDTuning(String cmd) {
  if (cmd == "T:ON") {
    pid_enabled = true;
    pid_integral = 0;
    pid_prev_error = 0;
    pid_last_time = millis();
    Serial.println("PID enabled");
    return;
  }
  if (cmd == "T:OFF") {
    pid_enabled = false;
    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, 0);
    Serial.println("PID disabled (open-loop fallback)");
    return;
  }

  // T:P:5.0 형식 파싱 (인덱스: T=0, :=1, P=2, :=3, 5.0=4~)
  if (cmd.length() < 5 || cmd.charAt(3) != ':') return;

  char param = cmd.charAt(2);
  float value = cmd.substring(4).toFloat();

  switch(param) {
    case 'P':
      PID_KP = value;
      Serial.print("PID Kp = "); Serial.println(PID_KP, 2);
      break;
    case 'I':
      PID_KI = value;
      pid_integral = 0;  // 적분 리셋
      Serial.print("PID Ki = "); Serial.println(PID_KI, 2);
      break;
    case 'D':
      PID_KD = value;
      Serial.print("PID Kd = "); Serial.println(PID_KD, 2);
      break;
    case 'B':
      PID_DEADBAND = value;
      Serial.print("PID Deadband = "); Serial.println(PID_DEADBAND, 2);
      break;
    default:
      Serial.println("Unknown PID param. Use P/I/D/B");
      break;
  }
}

// 조향 피드백 캘리브레이션 모드
// 시리얼에서 'K' 명령을 받으면 실행
void calibrate_steering_pot() {
  Serial.println("=== Steering Potentiometer Calibration ===");
  Serial.println("ADC: Higher = Left (120deg), Lower = Right (60deg)");
  Serial.println("1. Turn steering to FULL LEFT and send 'L'");
  Serial.println("2. Turn steering to CENTER and send 'C'");
  Serial.println("3. Turn steering to FULL RIGHT and send 'R'");
  Serial.println("4. Send 'X' to exit calibration");
  Serial.println("Press any other key to see current ADC value");

  bool calibrating = true;
  while (calibrating) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      int raw = analogRead(STEERING_POT_PIN);
      float voltage = (raw / 1023.0) * 2.56;

      switch(c) {
        case 'L':
          POT_LEFT = raw;
          Serial.print("LEFT set to: ");
          Serial.print(raw);
          Serial.print(" (");
          Serial.print(voltage, 2);
          Serial.println("V)");
          break;
        case 'C':
          POT_CENTER = raw;
          Serial.print("CENTER set to: ");
          Serial.print(raw);
          Serial.print(" (");
          Serial.print(voltage, 2);
          Serial.println("V)");
          break;
        case 'R':
          POT_RIGHT = raw;
          Serial.print("RIGHT set to: ");
          Serial.print(raw);
          Serial.print(" (");
          Serial.print(voltage, 2);
          Serial.println("V)");
          break;
        case 'X':
        case 'x':
          calibrating = false;
          Serial.println("Calibration complete!");
          Serial.print("POT_LEFT="); Serial.print(POT_LEFT);
          Serial.print(", POT_CENTER="); Serial.print(POT_CENTER);
          Serial.print(", POT_RIGHT="); Serial.println(POT_RIGHT);
          break;
        default:
          Serial.print("Current: ADC=");
          Serial.print(raw);
          Serial.print(", Voltage=");
          Serial.print(voltage, 2);
          Serial.print("V, Angle=");
          Serial.print(map_pot_to_angle(raw), 1);
          Serial.println(" deg");
          break;
      }
    }
    delay(100);
  }
}

// ==================== 명령 실행 (단일 문자) ====================
void executeCommand(char cmd) {
  command = cmd;
  switch(cmd) {
    case 'F':  // 전진 + 직진
      motor_forward(SPEED_FORWARD);
      steering_center();
      break;

    case 'B':  // 후진 + 직진
      motor_backward(SPEED_BACKWARD);
      steering_center();
      break;

    case 'L':  // 전진 + 최대 좌회전
      motor_forward(SPEED_TURN);
      steering_left_max();
      break;

    case 'R':  // 전진 + 최대 우회전
      motor_forward(SPEED_TURN);
      steering_right_max();
      break;

    case 'l':  // 전진 + 약한 좌회전
      motor_forward(SPEED_FORWARD);
      steering_left_soft();
      break;

    case 'r':  // 전진 + 약한 우회전
      motor_forward(SPEED_FORWARD);
      steering_right_soft();
      break;

    case 'S':  // 정지 + 중앙
      motor_stop();
      steering_center();
      break;

    case 'K':  // 캘리브레이션 모드 진입 (NEW)
      motor_stop();
      calibrate_steering_pot();
      break;

    case 'D':  // 디버그 정보 출력 (NEW)
      print_debug_info();
      break;

    default:
      // 알 수 없는 명령은 무시 (이전 상태 유지)
      break;
  }
}

// ==================== DC 모터 제어 함수 (차동 구동) ====================

// 좌측 바퀴 전진 (반대 방향으로 수정)
void motorLeft_forward(int speed) {
  analogWrite(motorLeft_IN1, 0);
  analogWrite(motorLeft_IN2, speed);
}

// 좌측 바퀴 후진 (반대 방향으로 수정)
void motorLeft_backward(int speed) {
  analogWrite(motorLeft_IN1, speed);
  analogWrite(motorLeft_IN2, 0);
}

// 좌측 바퀴 정지
void motorLeft_stop() {
  analogWrite(motorLeft_IN1, 0);
  analogWrite(motorLeft_IN2, 0);
}

// 우측 바퀴 전진
void motorRight_forward(int speed) {
  analogWrite(motorRight_IN1, speed);
  analogWrite(motorRight_IN2, 0);
}

// 우측 바퀴 후진
void motorRight_backward(int speed) {
  analogWrite(motorRight_IN1, 0);
  analogWrite(motorRight_IN2, speed);
}

// 우측 바퀴 정지
void motorRight_stop() {
  analogWrite(motorRight_IN1, 0);
  analogWrite(motorRight_IN2, 0);
}

// ==================== 통합 모터 제어 함수 ====================

// 전진 (양쪽 바퀴 같은 속도)
void motor_forward(int speed) {
  motorLeft_forward(speed);
  motorRight_forward(speed);
}

// 후진 (양쪽 바퀴 같은 속도)
void motor_backward(int speed) {
  motorLeft_backward(speed);
  motorRight_backward(speed);
}

// 정지 (양쪽 바퀴 모두 정지)
void motor_stop() {
  motorLeft_stop();
  motorRight_stop();
}

// ==================== 서보 조향 제어 함수 (최적화) ====================

// 서보 각도 설정 (PID 모드 + 오픈루프 폴백)
void setServoAngle(int angle) {
  target_steering_angle = angle;  // 목표 각도 업데이트

  if (pid_enabled) {
    // PID 모드: target만 설정, pid_steering_control()이 모터 제어
    return;
  }

  // 폴백: PID 비활성 시 기존 오픈루프 방식 (모터 드라이버 직접 제어)
  // 새 매핑: angle < 90 = 우회전(60°), angle > 90 = 좌회전(120°)
  if (angle < 90) {
    // 우회전 방향 (angle 감소)
    int power = map(90 - angle, 0, 90, 0, 255);
    analogWrite(servo_IN1, power);
    analogWrite(servo_IN2, 0);
  }
  else if (angle > 90) {
    // 좌회전 방향 (angle 증가)
    int power = map(angle - 90, 0, 90, 0, 255);
    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, power);
  }
  else {
    // 중앙 (정지)
    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, 0);
  }
}

// 중앙 (직진)
void steering_center() {
  setServoAngle(SERVO_CENTER);
}

// 최대 좌회전
void steering_left_max() {
  setServoAngle(SERVO_LEFT_MAX);
}

// 최대 우회전
void steering_right_max() {
  setServoAngle(SERVO_RIGHT_MAX);
}

// 약한 좌회전
void steering_left_soft() {
  setServoAngle(SERVO_LEFT_SOFT);
}

// 약한 우회전
void steering_right_soft() {
  setServoAngle(SERVO_RIGHT_SOFT);
}

// ==================== 초음파 센서 함수 (최적화) ====================

// 특정 센서 거리 측정 (HC-SR04)
long measure_ultrasonic(int index) {
  // Trig 핀으로 10us 펄스 전송
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[index], LOW);

  // Echo 핀에서 반사파 수신 (타임아웃 10ms로 축소)
  long duration = pulseIn(echoPins[index], HIGH, 2000);

  // 거리 계산: duration(us) * 0.034(cm/us) / 2
  long distance = duration * 0.034 / 2;

  // 유효 범위 체크 (타임아웃 축소로 170cm까지)
  if (distance < 2 || distance > 170) {
    distance = 0;  // 측정 실패 또는 범위 초과
  }

  return distance;
}

// 6개 센서 모두 측정
void measure_all_ultrasonic() {
  for (int i = 0; i < 6; i++) {
    distances[i] = measure_ultrasonic(i);
    delayMicroseconds(200);  // 센서 간 간섭 방지
  }
}

// Python으로 센서 데이터 전송
void send_sensor_data() {
  // 형식: "F:25,FL:30,FR:28,R:50,RL:45,RR:48"
  Serial.print("F:");    Serial.print(distances[FRONT]);
  Serial.print(",FL:");  Serial.print(distances[FRONT_LEFT]);
  Serial.print(",FR:");  Serial.print(distances[FRONT_RIGHT]);
  Serial.print(",R:");   Serial.print(distances[REAR]);
  Serial.print(",RL:");  Serial.print(distances[REAR_LEFT]);
  Serial.print(",RR:");  Serial.print(distances[REAR_RIGHT]);
  Serial.println();  // 줄바꿈
}

// ==================== 디버그용 함수 ====================

// 전체 상태 정보 출력
void print_debug_info() {
  Serial.println("========== Debug Info ==========");

  // 초음파 센서 정보
  Serial.println("[Ultrasonic Sensors]");
  Serial.print("  Front:       "); Serial.print(distances[FRONT]);       Serial.println(" cm");
  Serial.print("  Front-Left:  "); Serial.print(distances[FRONT_LEFT]);  Serial.println(" cm");
  Serial.print("  Front-Right: "); Serial.print(distances[FRONT_RIGHT]); Serial.println(" cm");
  Serial.print("  Rear:        "); Serial.print(distances[REAR]);        Serial.println(" cm");
  Serial.print("  Rear-Left:   "); Serial.print(distances[REAR_LEFT]);   Serial.println(" cm");
  Serial.print("  Rear-Right:  "); Serial.print(distances[REAR_RIGHT]);  Serial.println(" cm");

  // 조향 정보
  int raw_adc = analogRead(STEERING_POT_PIN);
  float voltage = (raw_adc / 1023.0) * 2.56;
  Serial.println("[Steering]");
  Serial.print("  Target Angle:  "); Serial.print(target_steering_angle); Serial.println(" deg");
  Serial.print("  Actual Angle:  "); Serial.print(actual_steering_angle, 1); Serial.println(" deg");
  Serial.print("  Error:         "); Serial.print((float)target_steering_angle - actual_steering_angle, 1); Serial.println(" deg");
  Serial.print("  Raw ADC:       "); Serial.println(raw_adc);
  Serial.print("  Voltage:       "); Serial.print(voltage, 2); Serial.println(" V");

  // 모터 상태
  Serial.println("[Motor]");
  Serial.print("  PWM:           "); Serial.println(current_pwm);

  // 캘리브레이션 값
  Serial.println("[Calibration]");
  Serial.print("  POT_LEFT:   "); Serial.print(POT_LEFT);   Serial.println(" (~2.27V, ref=2.56V)");
  Serial.print("  POT_CENTER: "); Serial.print(POT_CENTER); Serial.println(" (~1.87V, ref=2.56V)");
  Serial.print("  POT_RIGHT:  "); Serial.print(POT_RIGHT);  Serial.println(" (~1.47V, ref=2.56V)");

  // PID 상태
  Serial.println("[PID Control]");
  Serial.print("  Enabled:    "); Serial.println(pid_enabled ? "YES" : "NO");
  Serial.print("  Kp:         "); Serial.println(PID_KP, 2);
  Serial.print("  Ki:         "); Serial.println(PID_KI, 2);
  Serial.print("  Kd:         "); Serial.println(PID_KD, 2);
  Serial.print("  Deadband:   "); Serial.print(PID_DEADBAND, 1); Serial.println(" deg");
  Serial.print("  Integral:   "); Serial.println(pid_integral, 2);
  Serial.print("  Prev Error: "); Serial.println(pid_prev_error, 1);

  // 현재 명령
  Serial.println("[Current Command]");
  Serial.print("  Command: "); Serial.println(command);

  Serial.println("================================");
}

// 센서 상태 출력 (시리얼 모니터 확인용) - 기존 호환성 유지
void print_sensor_debug() {
  print_debug_info();
}
