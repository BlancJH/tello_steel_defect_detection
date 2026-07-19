import time

import pytest
from std_msgs.msg import Empty

from tello_defect_pipeline.tello_keyboard_controller_node import (
    TelloKeyboardControllerNode,
    VelocityCommand,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def info(self, msg):
        del msg

    def warn(self, msg):
        del msg


def make_controller():
    controller = TelloKeyboardControllerNode.__new__(TelloKeyboardControllerNode)
    controller.linear_speed = 0.35
    controller.vertical_speed = 0.45
    controller.yaw_speed = 0.55
    controller.movement_timeout = 0.25
    controller.current_command = VelocityCommand()
    controller.last_movement_time = None
    controller.cmd_publisher = FakePublisher()
    controller.takeoff_publisher = FakePublisher()
    controller.land_publisher = FakePublisher()
    controller.get_logger = lambda: FakeLogger()
    return controller


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        ("w", VelocityCommand(linear_z=0.45)),
        ("s", VelocityCommand(linear_z=-0.45)),
        ("a", VelocityCommand(angular_z=0.55)),
        ("d", VelocityCommand(angular_z=-0.55)),
        ("8", VelocityCommand(linear_x=0.35)),
        ("5", VelocityCommand(linear_x=-0.35)),
        ("4", VelocityCommand(linear_y=0.35)),
        ("6", VelocityCommand(linear_y=-0.35)),
    ],
)
def test_movement_keys_publish_expected_twist(key, expected):
    controller = make_controller()

    assert TelloKeyboardControllerNode.handle_key(controller, key) is True

    assert controller.current_command == expected
    published = controller.cmd_publisher.messages[-1]
    assert published.linear.x == expected.linear_x
    assert published.linear.y == expected.linear_y
    assert published.linear.z == expected.linear_z
    assert published.angular.z == expected.angular_z


def test_takeoff_key_publishes_hover_then_takeoff():
    controller = make_controller()

    assert TelloKeyboardControllerNode.handle_key(controller, "u") is True

    assert controller.current_command == VelocityCommand()
    assert len(controller.cmd_publisher.messages) == 1
    assert isinstance(controller.takeoff_publisher.messages[-1], Empty)


def test_land_key_publishes_hover_then_land():
    controller = make_controller()
    controller.current_command = VelocityCommand(linear_x=0.35)

    assert TelloKeyboardControllerNode.handle_key(controller, "j") is True

    assert controller.current_command == VelocityCommand()
    assert len(controller.cmd_publisher.messages) == 1
    assert isinstance(controller.land_publisher.messages[-1], Empty)


def test_quit_key_returns_false_after_hover():
    controller = make_controller()
    controller.current_command = VelocityCommand(linear_x=0.35)

    assert TelloKeyboardControllerNode.handle_key(controller, "q") is False

    assert controller.current_command == VelocityCommand()
    assert len(controller.cmd_publisher.messages) == 1


def test_movement_timeout_returns_to_hover():
    controller = make_controller()
    controller.current_command = VelocityCommand(linear_x=0.35)
    controller.last_movement_time = time.monotonic() - 1.0

    TelloKeyboardControllerNode.publish_current_command(controller)

    assert controller.current_command == VelocityCommand()
    published = controller.cmd_publisher.messages[-1]
    assert published.linear.x == 0.0
    assert published.linear.y == 0.0
    assert published.linear.z == 0.0
    assert published.angular.z == 0.0
