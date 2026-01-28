#!/usr/bin/env python3
"""Arduino serial bridge: command output + ultrasonic input.

Arduino 명령어:
  - F: 전진 + 직진
  - B: 후진 + 직진
  - L: 전진 + 최대 좌회전
  - R: 전진 + 최대 우회전
  - l: 전진 + 약한 좌회전
  - r: 전진 + 약한 우회전
  - S: 정지
"""

from __future__ import annotations

import math
from typing import Dict, List

import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("pyserial is required for arduino_bridge_node") from exc


SENSOR_ORDER = ("F", "FL", "FR", "R", "RL", "RR")

# 조향 각도 임계값 (도)
STEER_THRESHOLD_HARD = 15.0  # 최대 좌/우회전 (L, R)
STEER_THRESHOLD_SOFT = 5.0   # 약한 좌/우회전 (l, r)


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def parse_ultrasonic_line(line: str) -> Dict[str, float]:
    """Parse 'F:25,FL:30,...' into meters."""
    data: Dict[str, float] = {}
    for part in line.split(","):
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        key = key.strip()
        try:
            cm = float(value.strip())
        except ValueError:
            continue
        data[key] = cm / 100.0
    return data


class ArduinoBridgeNode:
    def __init__(self) -> None:
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baudrate = int(rospy.get_param("~baudrate", 9600))
        self.command_topic = rospy.get_param("~command_topic", "/arduino/cmd")
        self.ultrasonic_topic = rospy.get_param("~ultrasonic_topic", "/ultrasonic/ranges")
        self.use_legacy_cmd = bool(rospy.get_param("~use_legacy_cmd", True))
        self.max_speed_mps = float(rospy.get_param("~max_speed_mps", 2.0))
        self.max_steer_deg = float(rospy.get_param("~max_steer_deg", 30.0))
        self.center_servo_deg = float(rospy.get_param("~center_servo_deg", 90.0))

        # 시리얼 타임아웃 최적화 (Arduino 10ms 루프에 맞춤)
        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.ultra_pub = rospy.Publisher(self.ultrasonic_topic, Float32MultiArray, queue_size=1)
        rospy.Subscriber(self.command_topic, AckermannDrive, self.cmd_cb, queue_size=1)

        # 마지막 전송 명령 (중복 전송 방지)
        self.last_cmd = ""

        rospy.loginfo("[arduino_bridge] port=%s baud=%d legacy=%s",
                      self.port, self.baudrate, self.use_legacy_cmd)

    def cmd_cb(self, msg: AckermannDrive) -> None:
        if self.use_legacy_cmd:
            cmd = self._to_legacy_command(msg)
            # 중복 명령 전송 방지 (시리얼 버퍼 과부하 방지)
            if cmd != self.last_cmd:
                self.serial.write((cmd + "\n").encode("utf-8"))
                self.last_cmd = cmd
            return

        pwm = self._speed_to_pwm(msg.speed)
        servo = self._steer_to_servo(msg.steering_angle)
        payload = f"V:{pwm},S:{servo}\n"
        self.serial.write(payload.encode("utf-8"))

    def _to_legacy_command(self, msg: AckermannDrive) -> str:
        """AckermannDrive 메시지를 Arduino 명령어로 변환.

        명령어:
          - F: 전진 + 직진
          - B: 후진 + 직진
          - L: 전진 + 최대 좌회전 (steer < -15도)
          - R: 전진 + 최대 우회전 (steer > 15도)
          - l: 전진 + 약한 좌회전 (-15도 < steer < -5도)
          - r: 전진 + 약한 우회전 (5도 < steer < 15도)
          - S: 정지
        """
        speed = msg.speed
        steer = msg.steering_angle

        # 정지
        if abs(speed) < 1e-3:
            return "S"

        steer_deg = math.degrees(steer)

        # 후진 (조향 없이 직진 후진만 지원)
        if speed < 0:
            return "B"

        # 전진 + 조향
        if steer_deg > STEER_THRESHOLD_HARD:
            return "R"  # 최대 우회전
        if steer_deg < -STEER_THRESHOLD_HARD:
            return "L"  # 최대 좌회전
        if steer_deg > STEER_THRESHOLD_SOFT:
            return "r"  # 약한 우회전
        if steer_deg < -STEER_THRESHOLD_SOFT:
            return "l"  # 약한 좌회전

        return "F"  # 직진

    def _speed_to_pwm(self, speed: float) -> int:
        speed = clamp(speed, -self.max_speed_mps, self.max_speed_mps)
        ratio = abs(speed) / max(self.max_speed_mps, 1e-3)
        return int(round(clamp(ratio * 255.0, 0.0, 255.0)))

    def _steer_to_servo(self, steering_angle: float) -> int:
        steer_deg = math.degrees(steering_angle)
        steer_deg = clamp(steer_deg, -self.max_steer_deg, self.max_steer_deg)
        return int(round(self.center_servo_deg + steer_deg))

    def spin(self) -> None:
        # Arduino 10ms 루프에 맞춰 100Hz로 증가
        rate = rospy.Rate(100)
        buffer = ""
        while not rospy.is_shutdown():
            try:
                # 버퍼에 있는 데이터만 빠르게 읽기
                waiting = self.serial.in_waiting
                if waiting > 0:
                    data = self.serial.read(waiting)
                else:
                    rate.sleep()
                    continue
            except serial.SerialException as exc:
                rospy.logerr_throttle(1.0, "[arduino_bridge] serial error: %s", exc)
                rate.sleep()
                continue

            if data:
                try:
                    buffer += data.decode("utf-8", errors="ignore")
                except Exception:
                    buffer = ""

                # 버퍼 오버플로우 방지
                if len(buffer) > 1024:
                    buffer = buffer[-512:]

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    parsed = parse_ultrasonic_line(line)
                    if parsed:
                        msg = Float32MultiArray()
                        msg.data = [parsed.get(key, 0.0) for key in SENSOR_ORDER]
                        self.ultra_pub.publish(msg)

            rate.sleep()


def main() -> None:
    rospy.init_node("arduino_bridge_node")
    node = ArduinoBridgeNode()
    node.spin()


if __name__ == "__main__":
    main()
