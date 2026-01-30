/*
 * Ackermann Drive Arduino Firmware
 * ROS2 Humble - Continuous Mode (V:pwm,S:servo)
 *
 * 시리얼 통신: 115200 baud
 * 명령 형식: V:100,S:90 (속도:100, 조향각:90도)
 *
 * 하드웨어 연결:
 * - 조향 서보: PIN 9
 * - ESC/모터: PIN 10
 * - 초음파 센서 (선택): TRIG 7, ECHO 8
 */

#include <Servo.h>

// ========== 핀 설정 ==========
#define STEERING_SERVO_PIN 9
#define ESC_PIN 10
#define ULTRASONIC_TRIG_PIN 7
#define ULTRASONIC_ECHO_PIN 8

// ========== 서보 객체 ==========
Servo steeringServo;
Servo esc;

// ========== 파라미터 설정 ==========
// 조향 서보 설정 (ROS2 파라미터와 일치시켜야 함)
const int CENTER_SERVO_DEG = 90;      // 중앙 각도
const int MAX_STEER_DEG = 30;         // 최대 조향 각도 (좌우)
// ROS2 설정: center_servo_deg=90, max_steer_deg=30
// 결과 범위: 60도(좌) ~ 90도(중앙) ~ 120도(우)

// ESC 설정 (일반적인 RC ESC)
const int ESC_NEUTRAL = 90;           // 정지
const int ESC_FORWARD_MIN = 91;       // 전진 시작
const int ESC_FORWARD_MAX = 180;      // 전진 최대

// 주의: 현재 ROS2 코드는 항상 양수 PWM(0~255)만 전송
// 후진은 별도 처리 필요 (또는 ROS2 코드 수정 필요)

// 또는 일반 DC 모터 드라이버 사용 시 (L298N 등)
const bool USE_RC_ESC = true;         // true: RC ESC, false: DC 모터 드라이버

// ========== 변수 ==========
String inputString = "";
bool stringComplete = false;

int currentSpeed = 0;                 // -255 ~ 255
int currentSteering = CENTER_SERVO_DEG; // 조향각 (도)

unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 500; // 500ms

void setup() {
  // 시리얼 통신 시작
  Serial.begin(115200);

  // 서보 초기화
  steeringServo.attach(STEERING_SERVO_PIN);
  steeringServo.write(CENTER_SERVO_DEG);

  // ESC/모터 초기화
  if (USE_RC_ESC) {
    esc.attach(ESC_PIN);
    esc.write(ESC_NEUTRAL); // 정지 상태
    delay(1000); // ESC 초기화 대기
  } else {
    pinMode(ESC_PIN, OUTPUT);
    analogWrite(ESC_PIN, 0); // 정지
  }

  // 초음파 센서 초기화
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  // 준비 완료 메시지
  Serial.println("Arduino Ready: Ackermann Drive Firmware");
  Serial.println("Format: V:speed,S:angle");

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

  // 타임아웃 체크 (명령 없으면 정지)
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    emergencyStop();
  }

  // 초음파 센서 읽기 (100ms마다)
  static unsigned long lastUltrasonicRead = 0;
  if (millis() - lastUltrasonicRead > 100) {
    readUltrasonic();
    lastUltrasonicRead = millis();
  }
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
  // ROS2 형식: V:100,S:90
  // V: 속도 (0 ~ 255, 항상 양수 - ROS2 speedToPwm이 abs 사용)
  // S: 조향각 (60 ~ 120도, 중앙 90도)

  cmd.trim();

  int vIndex = cmd.indexOf("V:");
  int sIndex = cmd.indexOf("S:");
  int commaIndex = cmd.indexOf(',');

  if (vIndex != -1 && sIndex != -1 && commaIndex != -1) {
    // 속도 파싱 (0~255)
    String speedStr = cmd.substring(vIndex + 2, commaIndex);
    currentSpeed = speedStr.toInt();

    // 조향각 파싱 (60~120)
    String steerStr = cmd.substring(sIndex + 2);
    currentSteering = steerStr.toInt();

    // 제한
    currentSpeed = constrain(currentSpeed, 0, 255);  // 0~255 (양수만)
    currentSteering = constrain(currentSteering,
                                 CENTER_SERVO_DEG - MAX_STEER_DEG,  // 60
                                 CENTER_SERVO_DEG + MAX_STEER_DEG); // 120

    // 모터 및 서보 제어
    setMotorSpeed(currentSpeed);
    setSteeringAngle(currentSteering);

    lastCommandTime = millis();

    // 디버그 출력 (필요시 활성화)
    // Serial.print("PWM: ");
    // Serial.print(currentSpeed);
    // Serial.print(", Servo: ");
    // Serial.println(currentSteering);
  }
}

// ========== 모터 속도 제어 ==========
void setMotorSpeed(int speed) {
  // ROS2는 항상 0~255 양수만 전송
  // speed: 0~255 (0=정지, 255=최대속도)

  if (USE_RC_ESC) {
    // RC ESC 모드
    int escValue;

    if (speed > 5) {  // 약간의 데드존
      // 전진 (0~255 → 91~180)
      escValue = map(speed, 0, 255, ESC_FORWARD_MIN, ESC_FORWARD_MAX);
      escValue = constrain(escValue, ESC_FORWARD_MIN, ESC_FORWARD_MAX);
    } else {
      // 정지
      escValue = ESC_NEUTRAL;
    }

    esc.write(escValue);
  } else {
    // DC 모터 드라이버 모드 (PWM 0~255)
    analogWrite(ESC_PIN, speed);

    // L298N 등 방향 핀이 있다면:
    // digitalWrite(DIR_PIN, HIGH);  // 항상 전진
  }
}

// ========== 조향각 제어 ==========
void setSteeringAngle(int angle) {
  steeringServo.write(angle);
}

// ========== 긴급 정지 ==========
void emergencyStop() {
  currentSpeed = 0;
  currentSteering = CENTER_SERVO_DEG;

  if (USE_RC_ESC) {
    esc.write(ESC_NEUTRAL);
  } else {
    analogWrite(ESC_PIN, 0);
  }

  steeringServo.write(CENTER_SERVO_DEG);
}

// ========== 초음파 센서 읽기 ==========
void readUltrasonic() {
  // 트리거 펄스 생성
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // 에코 시간 측정
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000); // 30ms 타임아웃

  // 거리 계산 (cm)
  float distance = duration * 0.034 / 2.0;

  // 유효 범위 체크 (2cm ~ 400cm)
  if (distance > 2 && distance < 400) {
    // ROS2로 전송 (선택)
    // Serial.print("ULTRASONIC:");
    // Serial.println(distance);
  }
}
