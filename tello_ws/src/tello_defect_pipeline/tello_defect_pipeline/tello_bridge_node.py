"""ROS 2 bridge for DJI Tello EDU video and manual velocity control."""

from __future__ import annotations

from typing import Optional

import rclpy
from djitellopy import Tello
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
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


class TelloBridgeNode(Node):
    """Publish Tello camera frames and forward Twist commands to RC control."""

    def __init__(self) -> None:
        super().__init__("tello_bridge_node")

        self.tello: Optional[Tello] = None
        self.frame_reader = None
        self.is_airborne = False

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("takeoff_topic", "/tello/takeoff")
        self.declare_parameter("land_topic", "/tello/land")
        self.declare_parameter("camera_frame_id", "tello_camera")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("rc_scale", 100.0)
        self.declare_parameter("rc_min", -100)
        self.declare_parameter("rc_max", 100)
        self.declare_parameter("empty_frame_log_throttle_sec", 5.0)

        image_topic = self.get_parameter("image_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        takeoff_topic = self.get_parameter("takeoff_topic").value
        land_topic = self.get_parameter("land_topic").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        qos_depth = int(self.get_parameter("qos_depth").value)
        self.rc_scale = float(self.get_parameter("rc_scale").value)
        self.rc_min = int(self.get_parameter("rc_min").value)
        self.rc_max = int(self.get_parameter("rc_max").value)
        self.empty_frame_log_throttle_sec = float(
            self.get_parameter("empty_frame_log_throttle_sec").value
        )

        self.image_publisher = self.create_publisher(Image, image_topic, qos_depth)
        self.cmd_subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            qos_depth,
        )
        self.takeoff_subscription = self.create_subscription(
            Empty,
            takeoff_topic,
            self.takeoff_callback,
            qos_depth,
        )
        self.land_subscription = self.create_subscription(
            Empty,
            land_topic,
            self.land_callback,
            qos_depth,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_frame)

        self.connect_tello()

    def _clamp_rc(self, value: float) -> int:
        """Convert a normalized velocity command to a Tello RC value."""
        return max(self.rc_min, min(self.rc_max, int(value * self.rc_scale)))

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
                throttle_duration_sec=self.empty_frame_log_throttle_sec,
            )
            return

        image_msg = _bgr_frame_to_image_msg(frame)
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.camera_frame_id
        self.image_publisher.publish(image_msg)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Forward ROS Twist commands to Tello RC control."""
        if self.tello is None:
            self.get_logger().warn("Ignoring /cmd_vel because Tello is not connected.")
            return

        if not self.is_airborne:
            return

        left_right = -self._clamp_rc(msg.linear.y)
        forward_back = self._clamp_rc(msg.linear.x)
        up_down = self._clamp_rc(msg.linear.z)
        yaw = -self._clamp_rc(msg.angular.z)

        self.tello.send_rc_control(left_right, forward_back, up_down, yaw)

    def takeoff_callback(self, msg: Empty) -> None:
        """Take off and hover when requested by the custom controller."""
        del msg
        if self.tello is None:
            self.get_logger().warn(
                "Ignoring takeoff command because Tello is not connected."
            )
            return

        self.get_logger().warn("Taking off Tello...")
        try:
            self.tello.takeoff()
            self.is_airborne = True
            self.tello.send_rc_control(0, 0, 0, 0)
        except Exception as exc:  # noqa: BLE001 - takeoff should log and continue.
            self.get_logger().error(f"Failed to take off Tello: {exc}")

    def land_callback(self, msg: Empty) -> None:
        """Land the Tello when requested by the custom controller."""
        del msg
        if self.tello is None:
            self.get_logger().warn(
                "Ignoring land command because Tello is not connected."
            )
            return

        self.get_logger().warn("Landing Tello...")
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.land()
            self.is_airborne = False
        except Exception as exc:  # noqa: BLE001 - land should log and continue.
            self.get_logger().error(f"Failed to land Tello: {exc}")

    def shutdown(self) -> None:
        """Stop motion and release the Tello connection."""
        if self.tello is None:
            return

        self.get_logger().info("Stopping Tello bridge...")
        try:
            if self.is_airborne:
                self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.streamoff()
        except Exception as exc:  # noqa: BLE001 - shutdown should log and continue.
            self.get_logger().warn(f"Error while stopping Tello: {exc}")
        finally:
            self.is_airborne = False
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
