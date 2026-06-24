"""Keyboard control helper for manual_keyboard mode.

This node is not a flight controller. It only adjusts the same high-level
reference signals consumed by FlightControllerNode. The controller blocks and
MotorMixer remain in the command path.
"""

from __future__ import annotations

import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


HELP_TEXT = """
manual_keyboard commands:
  r/f: altitude reference up/down
  w/s: pitch reference forward/back
  a/d: roll reference left/right
  q/e: yaw reference left/right
  x: emergency stop
  c: clear emergency stop
  h: show this help
"""


class KeyboardControlNode(Node):
    """Reads terminal keys and publishes reference deltas."""

    def __init__(self) -> None:
        super().__init__("keyboard_control")
        self.declare_parameter(
            "manual_reference_delta_topic",
            "/flight_controller/manual_reference_delta",
        )
        self.declare_parameter(
            "emergency_stop_topic",
            "/flight_controller/emergency_stop",
        )
        self.declare_parameter("altitude_step_m", 0.10)
        self.declare_parameter("attitude_step_deg", 1.0)
        self.declare_parameter("yaw_step_deg", 5.0)
        self._delta_pub = self.create_publisher(
            Twist,
            self.get_parameter("manual_reference_delta_topic").value,
            10,
        )
        self._estop_pub = self.create_publisher(
            Bool,
            self.get_parameter("emergency_stop_topic").value,
            10,
        )
        self._timer = self.create_timer(0.05, self._poll_keyboard)
        self._terminal_settings = None
        if sys.stdin.isatty():
            self._terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            print(HELP_TEXT)
        else:
            self.get_logger().warning("stdin is not a TTY; keyboard input is disabled.")

    def destroy_node(self) -> bool:
        if self._terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
        return super().destroy_node()

    def _poll_keyboard(self) -> None:
        if self._terminal_settings is None:
            return
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return
        key = sys.stdin.read(1)
        self._handle_key(key)

    def _handle_key(self, key: str) -> None:
        altitude_step = float(self.get_parameter("altitude_step_m").value)
        attitude_step = math.radians(float(self.get_parameter("attitude_step_deg").value))
        yaw_step = math.radians(float(self.get_parameter("yaw_step_deg").value))

        delta = Twist()
        if key == "r":
            delta.linear.z = altitude_step
        elif key == "f":
            delta.linear.z = -altitude_step
        elif key == "w":
            delta.linear.x = attitude_step
        elif key == "s":
            delta.linear.x = -attitude_step
        elif key == "a":
            delta.linear.y = attitude_step
        elif key == "d":
            delta.linear.y = -attitude_step
        elif key == "q":
            delta.angular.z = yaw_step
        elif key == "e":
            delta.angular.z = -yaw_step
        elif key == "x":
            self._publish_estop(True)
            return
        elif key == "c":
            self._publish_estop(False)
            return
        elif key == "h":
            print(HELP_TEXT)
            return
        else:
            return

        self._delta_pub.publish(delta)

    def _publish_estop(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self._estop_pub.publish(msg)
        self.get_logger().warning(f"Emergency stop set to {active}.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
