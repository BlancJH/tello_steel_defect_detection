"""Launch the annotated defect image viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/defect_detections/image", description="Annotated image topic to display."),
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="defect_image_view",
            output="screen",
            arguments=[LaunchConfiguration("image_topic")],
        ),
    ])
