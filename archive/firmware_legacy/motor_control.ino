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
char command = 'S';           // 수신한 명령
long distances[6];            // 6개 초음파 센서 거리값 (cm)
int current_steering_angle = 90;  // 현재 조향 각도

// ==================== 초기화 ====================
void setup() {
  Serial.begin(9600);

  // 좌측 바퀴 모터 핀 설정
  pinMode(motorLeft_IN1, OUTPUT);
  pinMode(motorLeft_IN2, OUTPUT);

  // 우측 바퀴 모터 핀 설정
  pinMode(motorRight_IN1, OUTPUT);
  pinMode(motorRight_IN2, OUTPUT);

  // 서보모터 핀 설정 (모터 드라이버로 제어)
  pinMode(servo_IN1, OUTPUT);
  pinMode(servo_IN2, OUTPUT);

  // 초음파 센서 6개 핀 설정
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // 초기 상태: 정지, 직진
  motor_stop();
  steering_center();

  Serial.println("Arduino Ready!");
  Serial.println("DC Motor + 6 Ultrasonic Sensors + Servo Steering");
}

// ==================== 메인 루프 ====================
void loop() {
  // 1. Python에서 명령 수신
  if (Serial.available() > 0) {
    command = Serial.read();
  }

  // 2. 명령에 따라 모터 및 조향 제어
  executeCommand(command);

  // 3. 초음파 센서 6개 거리 측정 및 전송
  measure_all_ultrasonic();
  send_sensor_data();

  delay(50);  // 50ms 대기 (초당 20회 업데이트)
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

    default:   // 알 수 없는 명령 → 정지
      motor_stop();
      steering_center();
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

// ==================== 서보 조향 제어 함수 ====================
// 주의: 서보모터를 모터 드라이버로 제어하는 경우
// 일반적으로 서보모터는 PWM 신호로 제어하지만,
// 모터 드라이버를 사용하는 경우 다르게 동작할 수 있습니다.

// 서보 각도 설정 (모터 드라이버 제어 방식) - 방향 수정
void setServoAngle(int angle) {
  // 각도를 PWM 듀티로 변환 (0~255)
  // 중앙(90도) = 중립, 작으면 왼쪽, 크면 오른쪽

  if (angle < 90) {
    // 왼쪽 회전 (IN2 방향으로 수정)
    int power = map(90 - angle, 0, 90, 0, 255);
    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, power);
  }
  else if (angle > 90) {
    // 오른쪽 회전 (IN1 방향으로 수정)
    int power = map(angle - 90, 0, 90, 0, 255);
    analogWrite(servo_IN1, power);
    analogWrite(servo_IN2, 0);
  }
  else {
    // 중앙 (정지)
    analogWrite(servo_IN1, 0);
    analogWrite(servo_IN2, 0);
  }

  current_steering_angle = angle;

  // 서보가 움직일 시간 대기
  delay(50);
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

// ==================== 초음파 센서 함수 ====================

// 특정 센서 거리 측정 (HC-SR04)
long measure_ultrasonic(int index) {
  // Trig 핀으로 10us 펄스 전송
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[index], LOW);

  // Echo 핀에서 반사파 수신 (타임아웃 30ms)
  long duration = pulseIn(echoPins[index], HIGH, 30000);

  // 거리 계산: duration(us) * 0.034(cm/us) / 2
  long distance = duration * 0.034 / 2;

  // 유효 범위 체크 (HC-SR04: 2~400cm)
  if (distance < 2 || distance > 400) {
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

// 센서 상태 출력 (시리얼 모니터 확인용)
void print_sensor_debug() {
  Serial.println("===== Sensor Status =====");
  Serial.print("Front:       "); Serial.print(distances[FRONT]);       Serial.println(" cm");
  Serial.print("Front-Left:  "); Serial.print(distances[FRONT_LEFT]);  Serial.println(" cm");
  Serial.print("Front-Right: "); Serial.print(distances[FRONT_RIGHT]); Serial.println(" cm");
  Serial.print("Rear:        "); Serial.print(distances[REAR]);        Serial.println(" cm");
  Serial.print("Rear-Left:   "); Serial.print(distances[REAR_LEFT]);   Serial.println(" cm");
  Serial.print("Rear-Right:  "); Serial.print(distances[REAR_RIGHT]);  Serial.println(" cm");
  Serial.print("Steering:    "); Serial.print(current_steering_angle); Serial.println(" deg");
  Serial.println("========================");
}
