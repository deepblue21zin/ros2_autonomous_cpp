#include <Car_Library.h>

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

// 가변저항 캘리브레이션 값 (실측값 기반)
// 전압: 왼쪽 2.53V, 중앙 2.2V, 오른쪽 1.79V
// ADC = (전압 / 5.0) * 1023
int POT_LEFT = 518;               // 최대 좌회전 시 ADC 값 (2.53V)
int POT_CENTER = 450;             // 중앙(직진) 시 ADC 값 (2.2V)
int POT_RIGHT = 366;              // 최대 우회전 시 ADC 값 (1.79V)

// 조향각 범위 (도)
float ANGLE_LEFT = 60.0;          // 최대 좌회전 각도
float ANGLE_CENTER_DEG = 90.0;    // 중앙 각도
float ANGLE_RIGHT = 120.0;        // 최대 우회전 각도

// 피드백 필터링
int pot_readings[5];              // 이동평균 필터 버퍼
int pot_index = 0;
bool pot_buffer_filled = false;

// 피드백 데이터 전송 주기 설정
bool ENABLE_STEERING_FEEDBACK = true;   // 피드백 기능 활성화 여부
int FEEDBACK_SEND_INTERVAL = 10;        // N번 루프마다 피드백 전송 (5 = 센서데이터와 동일 주기)

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

// 서보 조향 각도 (실제 차량에 맞게 조정 필요)
int SERVO_CENTER = 90;        // 중앙 (직진)
int SERVO_LEFT_MAX = 60;      // 최대 좌회전 각도
int SERVO_RIGHT_MAX = 120;    // 최대 우회전 각도
int SERVO_LEFT_SOFT = 75;     // 약한 좌회전
int SERVO_RIGHT_SOFT = 105;   // 약한 우회전

// ==================== 전역 변수 ====================
char command = 'S';               // 수신한 명령
long distances[6];                // 6개 초음파 센서 거리값 (cm)
int target_steering_angle = 90;   // 목표 조향 각도 (명령으로 설정된 값)
float actual_steering_angle = 90.0;  // 실제 조향 각도 (가변저항에서 읽은 값)
int loop_count = 0;               // 루프 카운터 (초음파 측정 빈도 조절용)
int feedback_count = 0;           // 피드백 전송 카운터

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

  // 초음파 센서 6개 핀 설정
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // 이동평균 필터 버퍼 초기화 (중앙값으로)
  for (int i = 0; i < 5; i++) {
    pot_readings[i] = POT_CENTER;  // 450 (2.2V)
  }

  // 초기 상태: 정지, 직진
  motor_stop();
  steering_center();

  Serial.println("Arduino Ready!");
  Serial.println("DC Motor + 6 Ultrasonic + Servo + Steering Feedback");
  Serial.println("Steering feedback enabled: potentiometer on A0");
}

// ==================== 메인 루프 (최적화) ====================
void loop() {
  // 1. 명령 수신 (줄바꿈/캐리지리턴 무시)
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      command = c;
    }
  }

  // 2. 명령 실행 (매 루프)
  executeCommand(command);

  // 3. 조향각 피드백 읽기 (매 루프 - 주행에 영향 없음)
  if (ENABLE_STEERING_FEEDBACK) {
    read_steering_feedback();
  }

  // 4. 초음파 측정은 5회 중 1회만 (딜레이 감소)
  loop_count++;
  if (loop_count >= 5) {
    measure_all_ultrasonic();
    send_sensor_data();
    loop_count = 0;
  }

  // 5. 조향 피드백 전송 (별도 주기)
  feedback_count++;
  if (ENABLE_STEERING_FEEDBACK && feedback_count >= FEEDBACK_SEND_INTERVAL) {
    send_steering_feedback();
    feedback_count = 0;
  }

  delay(10);  // 10ms 루프
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
// ADC 높을수록 왼쪽(60°), 낮을수록 오른쪽(120°)
float map_pot_to_angle(int pot_value) {
  // 범위 제한 (RIGHT가 더 작은 값)
  pot_value = constrain(pot_value, POT_RIGHT, POT_LEFT);

  // 선형 보간
  float angle;
  if (pot_value >= POT_CENTER) {
    // 좌회전 영역: ADC 높음 (POT_CENTER ~ POT_LEFT -> 90° ~ 60°)
    angle = map_float(pot_value, POT_CENTER, POT_LEFT, ANGLE_CENTER_DEG, ANGLE_LEFT);
  } else {
    // 우회전 영역: ADC 낮음 (POT_RIGHT ~ POT_CENTER -> 120° ~ 90°)
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
  float voltage = (raw_adc / 1023.0) * 5.0;
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

// 조향 피드백 캘리브레이션 모드
// 시리얼에서 'K' 명령을 받으면 실행
void calibrate_steering_pot() {
  Serial.println("=== Steering Potentiometer Calibration ===");
  Serial.println("ADC: Higher = Left, Lower = Right");
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
      float voltage = (raw / 1023.0) * 5.0;

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

// ==================== 명령 실행 ====================
void executeCommand(char cmd) {
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

// 서보 각도 설정 (모터 드라이버 제어 방식)
void setServoAngle(int angle) {
  if (angle < 90) {
    // 왼쪽 회전
    int power = map(90 - angle, 0, 90, 0, 255);
    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, power);
  }
  else if (angle > 90) {
    // 오른쪽 회전
    int power = map(angle - 90, 0, 90, 0, 255);
    analogWrite(servo_IN1, power);
    analogWrite(servo_IN2, 0);
  }
  else {
    // 중앙 (정지)
    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, 0);
  }

  target_steering_angle = angle;  // 목표 각도 업데이트
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
  long duration = pulseIn(echoPins[index], HIGH, 10000);

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
  float voltage = (raw_adc / 1023.0) * 5.0;
  Serial.println("[Steering]");
  Serial.print("  Target Angle:  "); Serial.print(target_steering_angle); Serial.println(" deg");
  Serial.print("  Actual Angle:  "); Serial.print(actual_steering_angle, 1); Serial.println(" deg");
  Serial.print("  Error:         "); Serial.print((float)target_steering_angle - actual_steering_angle, 1); Serial.println(" deg");
  Serial.print("  Raw ADC:       "); Serial.println(raw_adc);
  Serial.print("  Voltage:       "); Serial.print(voltage, 2); Serial.println(" V");

  // 캘리브레이션 값
  Serial.println("[Calibration]");
  Serial.print("  POT_LEFT:   "); Serial.print(POT_LEFT);   Serial.println(" (2.53V)");
  Serial.print("  POT_CENTER: "); Serial.print(POT_CENTER); Serial.println(" (2.20V)");
  Serial.print("  POT_RIGHT:  "); Serial.print(POT_RIGHT);  Serial.println(" (1.79V)");

  // 현재 명령
  Serial.println("[Current Command]");
  Serial.print("  Command: "); Serial.println(command);

  Serial.println("================================");
}

// 센서 상태 출력 (시리얼 모니터 확인용) - 기존 호환성 유지
void print_sensor_debug() {
  print_debug_info();
}
