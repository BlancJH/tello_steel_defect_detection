"""ROS 2 bridge for DJI Tello EDU video and manual velocity control."""

from __future__ import annotations

from typing import Optional

import rclpy
from djitellopy import Tello
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


def _bgr_frame_to_image_msg(frame) -> Image:
    """Convert a contiguous BGR image into a ROS Image message."""
    contiguous = frame if frame.flags["C_CONTIGUOUS"] else frame.copy()
    msg = Image()
    msg.height = contiguous.shape[0]
    msg.width = contiguous.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = contiguous.shape[1] * 3
    msg.data = contiguous.tobytes()
    return msg


def _clamp_rc(value: float) -> int:
    """Convert a normalized velocity command to a Tello RC value."""
    return max(-100, min(100, int(value * 100)))


class TelloBridgeNode(Node):
    """Publish Tello camera frames and forward Twist commands to RC control."""

    def __init__(self) -> None:
        super().__init__("tello_bridge_node")

        self.tello: Optional[Tello] = None
        self.frame_reader = None

        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.cmd_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

        self.connect_tello()

    def connect_tello(self) -> None:
        """Connect to the drone and start the video stream."""
        self.tello = Tello()
        self.get_logger().info("Connecting to Tello...")
        self.tello.connect()
        self.get_logger().info("Starting Tello video stream...")
        self.tello.streamon()
        self.frame_reader = self.tello.get_frame_read()
        self.get_logger().info("Tello bridge is ready.")

    def publish_frame(self) -> None:
        """Read the latest Tello frame and publish it as a ROS Image."""
        if self.frame_reader is None:
            return

        frame = self.frame_reader.frame
        if frame is None:
            self.get_logger().warn(
                "Received an empty Tello frame.",
                throttle_duration_sec=5.0,
            )
            return

        image_msg = _bgr_frame_to_image_msg(frame)
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "tello_camera"
        self.image_publisher.publish(image_msg)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Forward ROS Twist commands to Tello RC control."""
        if self.tello is None:
            self.get_logger().warn("Ignoring /cmd_vel because Tello is not connected.")
            return

        left_right = _clamp_rc(msg.linear.y)
        forward_back = _clamp_rc(msg.linear.x)
        up_down = _clamp_rc(msg.linear.z)
        yaw = _clamp_rc(msg.angular.z)

        self.tello.send_rc_control(left_right, forward_back, up_down, yaw)

    def shutdown(self) -> None:
        """Stop motion and release the Tello connection."""
        if self.tello is None:
            return

        self.get_logger().info("Stopping Tello bridge...")
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.streamoff()
        except Exception as exc:  # noqa: BLE001 - shutdown should log and continue.
            self.get_logger().warn(f"Error while stopping Tello: {exc}")
        finally:
            self.tello.end()
            self.tello = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TelloBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
