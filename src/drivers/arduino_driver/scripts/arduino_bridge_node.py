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
import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("pyserial is required for arduino_bridge_node") from exc


SENSOR_ORDER = ("F", "FL", "FR", "R", "RL", "RR")

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


class ArduinoBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("arduino_bridge_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("command_topic", "/arduino/cmd")
        self.declare_parameter("ultrasonic_topic", "/ultrasonic/ranges")
        self.declare_parameter("max_speed_mps", 2.0)
        self.declare_parameter("max_steer_deg", 30.0)
        self.declare_parameter("center_servo_deg", 90.0)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.command_topic = self.get_parameter("command_topic").value
        self.ultrasonic_topic = self.get_parameter("ultrasonic_topic").value
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.max_steer_deg = float(self.get_parameter("max_steer_deg").value)
        self.center_servo_deg = float(self.get_parameter("center_servo_deg").value)

        # 시리얼 포트 열기 (DTR 토글로 Arduino 리셋 발생)
        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.serial_lock = threading.Lock()

        # Arduino 리셋 후 부팅 대기 (2초)
        self.get_logger().info("[arduino_bridge] Arduino 리셋 대기 중 (2초)...")
        import time
        time.sleep(2)
        # 부팅 메시지 비우기
        if self.serial.in_waiting:
            self.serial.read(self.serial.in_waiting)
        self.get_logger().info("[arduino_bridge] Arduino 준비 완료")

        self.ultra_pub = self.create_publisher(Float32MultiArray, self.ultrasonic_topic, 1)
        self.create_subscription(AckermannDrive, self.command_topic, self.cmd_cb, 1)

        # 마지막 전송 명령 (중복 전송 방지)
        self.last_cmd = ""
        self.buffer = ""

        self.read_timer = self.create_timer(0.01, self.read_serial)

        self.get_logger().info(
            f"[arduino_bridge] port={self.port} baud={self.baudrate} mode=continuous"
        )

    def cleanup(self) -> None:
        """Send stop command to Arduino on shutdown."""
        try:
            if self.serial and self.serial.is_open:
                self.get_logger().info("[arduino_bridge] Sending stop command...")
                stop_cmd = "V:0,S:90\n"
                with self.serial_lock:
                    self.serial.write(stop_cmd.encode("utf-8"))
                    self.serial.flush()
                import time
                time.sleep(0.1)  # 명령 전송 대기
                self.get_logger().info("[arduino_bridge] Motor stopped")
        except Exception as e:
            self.get_logger().error(f"[arduino_bridge] Cleanup error: {e}")

    def cmd_cb(self, msg: AckermannDrive) -> None:
        pwm = self._speed_to_pwm(msg.speed)
        servo = self._steer_to_servo(msg.steering_angle)
        payload = f"V:{pwm},S:{servo}\n"
        with self.serial_lock:
            self.serial.write(payload.encode("utf-8"))

    def _speed_to_pwm(self, speed: float) -> int:
        # Firmware expects 0~255 PWM (forward only)
        speed = clamp(speed, 0.0, self.max_speed_mps)
        ratio = speed / max(self.max_speed_mps, 1e-3)
        return int(round(clamp(ratio * 255.0, 0.0, 255.0)))

    def _steer_to_servo(self, steering_angle: float) -> int:
        steer_deg = math.degrees(steering_angle)
        steer_deg = clamp(steer_deg, -self.max_steer_deg, self.max_steer_deg)
        servo = self.center_servo_deg + steer_deg
        return int(round(clamp(servo, self.center_servo_deg - self.max_steer_deg,
                               self.center_servo_deg + self.max_steer_deg)))

    def read_serial(self) -> None:
        try:
            waiting = self.serial.in_waiting
            if waiting <= 0:
                return
            with self.serial_lock:
                data = self.serial.read(waiting)
        except serial.SerialException as exc:
            self.get_logger().error(f"[arduino_bridge] serial error: {exc}")
            return

        if not data:
            return

        try:
            self.buffer += data.decode("utf-8", errors="ignore")
        except Exception:
            self.buffer = ""

        # 버퍼 오버플로우 방지
        if len(self.buffer) > 1024:
            self.buffer = self.buffer[-512:]

        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            parsed = parse_ultrasonic_line(line)
            if parsed:
                msg = Float32MultiArray()
                msg.data = [parsed.get(key, 0.0) for key in SENSOR_ORDER]
                self.ultra_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # 모터 정지 명령 전송 (CRITICAL!)
            node.cleanup()
            # 시리얼 포트 닫기
            if node.serial and node.serial.is_open:
                node.serial.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
