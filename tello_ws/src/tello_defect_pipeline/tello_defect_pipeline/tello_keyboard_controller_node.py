"""Custom keyboard controller for Tello manual scanning."""

from __future__ import annotations

import select
import time
import sys
import termios
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty


HELP_TEXT = """
Custom Tello keyboard controller
--------------------------------
u : takeoff / hover
j : land
w : up (+z)
s : down (-z)
a : yaw left
d : yaw right
8 : pitch forward
5 : pitch backward
4 : roll left
6 : roll right

q : quit controller

Movement keys are active only while held.
Keep this terminal focused while flying.
"""


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_z: float = 0.0


class RawTerminal:
    """Temporarily switch stdin to raw mode for single-key reads."""

    def __enter__(self) -> "RawTerminal":
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def read_key(self, timeout_sec: float) -> Optional[str]:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not ready:
            return None
        return sys.stdin.read(1)


class TelloKeyboardControllerNode(Node):
    """Publish custom keyboard commands to the Tello bridge."""

    def __init__(self) -> None:
        super().__init__("tello_keyboard_controller_node")

        self.declare_parameter("linear_speed", 0.35)
        self.declare_parameter("vertical_speed", 0.45)
        self.declare_parameter("yaw_speed", 0.55)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("movement_timeout", 0.25)

        self.linear_speed = self.get_parameter("linear_speed").value
        self.vertical_speed = self.get_parameter("vertical_speed").value
        self.yaw_speed = self.get_parameter("yaw_speed").value
        publish_rate = self.get_parameter("publish_rate").value
        self.movement_timeout = self.get_parameter("movement_timeout").value

        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.takeoff_publisher = self.create_publisher(Empty, "/tello/takeoff", 10)
        self.land_publisher = self.create_publisher(Empty, "/tello/land", 10)
        self.current_command = VelocityCommand()
        self.last_movement_time: Optional[float] = None
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_current_command,
        )

        self.get_logger().info("Custom Tello keyboard controller ready.")

    def handle_key(self, key: str) -> bool:
        """Handle one keypress. Return False when the controller should quit."""
        if key == "q" or key == "":
            self.hover()
            return False

        if key == "u":
            self.takeoff()
        elif key == "j":
            self.hover()
            self.land_publisher.publish(Empty())
            self.get_logger().warn("Land command published on /tello/land.")
        elif key == "w":
            self.set_command(VelocityCommand(linear_z=self.vertical_speed), "up")
        elif key == "s":
            self.set_command(VelocityCommand(linear_z=-self.vertical_speed), "down")
        elif key == "a":
            self.set_command(VelocityCommand(angular_z=self.yaw_speed), "yaw left")
        elif key == "d":
            self.set_command(VelocityCommand(angular_z=-self.yaw_speed), "yaw right")
        elif key == "8":
            self.set_command(
                VelocityCommand(linear_x=self.linear_speed),
                "pitch forward",
            )
        elif key == "5":
            self.set_command(
                VelocityCommand(linear_x=-self.linear_speed),
                "pitch backward",
            )
        elif key == "4":
            self.set_command(VelocityCommand(linear_y=self.linear_speed), "roll left")
        elif key == "6":
            self.set_command(VelocityCommand(linear_y=-self.linear_speed), "roll right")
        else:
            self.get_logger().info(f"Unmapped key: {key!r}")

        return True

    def set_command(self, command: VelocityCommand, label: str) -> None:
        self.current_command = command
        self.last_movement_time = time.monotonic()
        self.publish_current_command()
        self.get_logger().info(f"Command while held: {label}")

    def hover(self) -> None:
        self.current_command = VelocityCommand()
        self.last_movement_time = None
        self.publish_current_command()
        self.get_logger().info("Command: hover")

    def takeoff(self) -> None:
        self.hover()
        self.takeoff_publisher.publish(Empty())
        self.get_logger().warn("Takeoff command published on /tello/takeoff.")

    def publish_current_command(self) -> None:
        if self.last_movement_time is not None:
            elapsed = time.monotonic() - self.last_movement_time
            if elapsed > self.movement_timeout:
                self.current_command = VelocityCommand()
                self.last_movement_time = None

        msg = Twist()
        msg.linear.x = self.current_command.linear_x
        msg.linear.y = self.current_command.linear_y
        msg.linear.z = self.current_command.linear_z
        msg.angular.z = self.current_command.angular_z
        self.cmd_publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TelloKeyboardControllerNode()
    print(HELP_TEXT)

    try:
        with RawTerminal() as terminal:
            running = True
            while rclpy.ok() and running:
                rclpy.spin_once(node, timeout_sec=0.01)
                key = terminal.read_key(timeout_sec=0.05)
                if key is not None:
                    running = node.handle_key(key)
    except KeyboardInterrupt:
        node.hover()
    finally:
        node.hover()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
