/*
 * Ackermann Drive Arduino Firmware for ROS2
 * 차동 구동 방식 (Differential Drive) + 서보 조향
 *
 * 시리얼 통신: 115200 baud
 * 명령 형식: V:pwm,S:angle (ROS2 연속 모드)
 *
 * 하드웨어:
 * - 좌측 모터 드라이버: PIN 2, 3
 * - 우측 모터 드라이버: PIN 4, 5
 * - 서보(조향) 드라이버: PIN 6, 7
 * - 초음파 센서 6개: TRIG(22,24,26,28,30,32), ECHO(23,25,27,29,31,33)
 */

#include <Arduino.h>

// ========== 핀 설정 (motor_control.ino와 동일) ==========
// 모터 드라이버 1: 좌측 바퀴
#define MOTOR_LEFT_IN1  2
#define MOTOR_LEFT_IN2  3

// 모터 드라이버 2: 우측 바퀴
#define MOTOR_RIGHT_IN1 4
#define MOTOR_RIGHT_IN2 5

// 모터 드라이버 3: 서보 조향
#define SERVO_IN1 6
#define SERVO_IN2 7

// 초음파 센서 6개
int trigPins[6] = {22, 24, 26, 28, 30, 32};
int echoPins[6] = {23, 25, 27, 29, 31, 33};

// 센서 인덱스
#define FRONT       0
#define FRONT_LEFT  1
#define FRONT_RIGHT 2
#define REAR        3
#define REAR_LEFT   4
#define REAR_RIGHT  5

// ========== 파라미터 설정 ==========
// ROS2 파라미터와 일치
const int CENTER_SERVO_DEG = 90;      // 직진
const int MAX_STEER_DEG = 30;         // 최대 조향각

// ========== 변수 ==========
String inputString = "";
bool stringComplete = false;

int currentSpeed = 0;                 // 0~255 PWM
int currentSteering = CENTER_SERVO_DEG; // 60~120도
long distances[6];                    // 초음파 거리 (cm)

unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 1000; // 1000ms (타임아웃 여유 증가)
bool commandReceived = false;         // 최초 명령 수신 플래그

void setup() {
  // 시리얼 통신
  Serial.begin(115200);

  // 모터 핀 설정
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  // 서보 핀 설정
  pinMode(SERVO_IN1, OUTPUT);
  pinMode(SERVO_IN2, OUTPUT);

  // 초음파 센서 핀 설정
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // 초기 상태: 정지, 직진
  motorStop();
  setSteeringAngle(CENTER_SERVO_DEG);

  Serial.println("Arduino Ready: ROS2 Ackermann Drive");
  Serial.println("Format: V:pwm,S:angle");

  inputString.reserve(50);
}

void loop() {
  // 시리얼 데이터 읽기
  serialEvent();

  // 명령 파싱 및 실행
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // 타임아웃 체크 (최초 명령 수신 후에만)
  if (commandReceived && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
    emergencyStop();
    commandReceived = false;  // 재시작 대기
  }

  // 초음파 센서 읽기 (100ms마다)
  static unsigned long lastUltrasonicRead = 0;
  if (millis() - lastUltrasonicRead > 100) {
    measureAllUltrasonic();
    sendSensorData();
    lastUltrasonicRead = millis();
  }

  delay(10);
}

// ========== 시리얼 이벤트 ==========
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        stringComplete = true;
      }
    } else {
      inputString += inChar;
    }
  }
}

// ========== 명령 파싱 ==========
void parseCommand(String cmd) {
  // ROS2 형식: V:255,S:86
  // V: 속도 PWM (0~255)
  // S: 조향각 (60~120도)

  cmd.trim();

  int vIndex = cmd.indexOf("V:");
  int sIndex = cmd.indexOf("S:");
  int commaIndex = cmd.indexOf(',');

  if (vIndex != -1 && sIndex != -1 && commaIndex != -1) {
    // 속도 파싱
    String speedStr = cmd.substring(vIndex + 2, commaIndex);
    currentSpeed = speedStr.toInt();

    // 조향각 파싱
    String steerStr = cmd.substring(sIndex + 2);
    currentSteering = steerStr.toInt();

    // 제한
    currentSpeed = constrain(currentSpeed, 0, 255);
    currentSteering = constrain(currentSteering,
                                 CENTER_SERVO_DEG - MAX_STEER_DEG,  // 60
                                 CENTER_SERVO_DEG + MAX_STEER_DEG); // 120

    // 모터 및 서보 제어
    setMotorSpeed(currentSpeed);
    setSteeringAngle(currentSteering);

    // 타임아웃 및 수신 플래그 업데이트
    lastCommandTime = millis();
    commandReceived = true;

    // 디버그 출력 (CMD: 접두사로 센서 데이터와 구분)
    Serial.print("CMD:");
    Serial.print(currentSpeed);
    Serial.print(",");
    Serial.println(currentSteering);
  } else {
    // 파싱 실패 디버그
    Serial.print("PARSE_FAIL:");
    Serial.println(cmd);
  }
}

// ========== 모터 속도 제어 (차동 구동) ==========
void setMotorSpeed(int speed) {
  // 0~255 PWM
  // 좌우 바퀴를 같은 속도로 제어 (Ackermann처럼)

  if (speed > 5) {
    // 전진
    motorForward(speed);
  } else {
    // 정지
    motorStop();
  }
}

// 전진 (양쪽 바퀴 같은 속도)
void motorForward(int speed) {
  // 좌측 바퀴 전진 (반대 방향으로 설정됨)
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, speed);

  // 우측 바퀴 전진
  analogWrite(MOTOR_RIGHT_IN1, speed);
  analogWrite(MOTOR_RIGHT_IN2, 0);
}

// 후진 (양쪽 바퀴 같은 속도)
void motorBackward(int speed) {
  // 좌측 바퀴 후진
  analogWrite(MOTOR_LEFT_IN1, speed);
  analogWrite(MOTOR_LEFT_IN2, 0);

  // 우측 바퀴 후진
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, speed);
}

// 정지
void motorStop() {
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
}

// ========== 조향각 제어 (모터 드라이버 방식) ==========
void setSteeringAngle(int angle) {
  // 60~120도 → 모터 드라이버 PWM으로 변환
  // 90도 = 중앙(정지)
  // <90도 = 왼쪽 회전
  // >90도 = 오른쪽 회전

  if (angle < 90) {
    // 왼쪽 회전
    int power = map(90 - angle, 0, 90, 0, 255);
    power = constrain(power, 0, 255);
    analogWrite(SERVO_IN1, 0);
    analogWrite(SERVO_IN2, power);
  }
  else if (angle > 90) {
    // 오른쪽 회전
    int power = map(angle - 90, 0, 90, 0, 255);
    power = constrain(power, 0, 255);
    analogWrite(SERVO_IN1, power);
    analogWrite(SERVO_IN2, 0);
  }
  else {
    // 중앙 (정지)
    analogWrite(SERVO_IN1, 0);
    analogWrite(SERVO_IN2, 0);
  }
}

// ========== 긴급 정지 ==========
void emergencyStop() {
  currentSpeed = 0;
  currentSteering = CENTER_SERVO_DEG;
  motorStop();
  setSteeringAngle(CENTER_SERVO_DEG);
}

// ========== 초음파 센서 읽기 ==========
long measureUltrasonic(int index) {
  // Trig 펄스
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[index], LOW);

  // Echo 수신 (타임아웃 10ms)
  long duration = pulseIn(echoPins[index], HIGH, 10000);

  // 거리 계산 (cm)
  long distance = duration * 0.034 / 2;

  // 유효 범위 체크 (2~170cm)
  if (distance < 2 || distance > 170) {
    distance = 0;
  }

  return distance;
}

// 6개 센서 모두 측정
void measureAllUltrasonic() {
  for (int i = 0; i < 6; i++) {
    distances[i] = measureUltrasonic(i);
    delayMicroseconds(200);  // 센서 간 간섭 방지
  }
}

// ROS2로 센서 데이터 전송
void sendSensorData() {
  // 형식: "F:25,FL:30,FR:28,R:50,RL:45,RR:48"
  Serial.print("F:");    Serial.print(distances[FRONT]);
  Serial.print(",FL:");  Serial.print(distances[FRONT_LEFT]);
  Serial.print(",FR:");  Serial.print(distances[FRONT_RIGHT]);
  Serial.print(",R:");   Serial.print(distances[REAR]);
  Serial.print(",RL:");  Serial.print(distances[REAR_LEFT]);
  Serial.print(",RR:");  Serial.print(distances[REAR_RIGHT]);
  Serial.println();
}
