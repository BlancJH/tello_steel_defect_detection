"""Launch the live Tello defect detection pipeline."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _venv_pythonpath_env():
    for parent in Path(__file__).resolve().parents:
        venv_lib = parent / "venv" / "lib"
        if not venv_lib.is_dir():
            continue

        site_packages = sorted(venv_lib.glob("python*/site-packages"))
        if site_packages:
            paths = [str(site_packages[0])]
            existing_pythonpath = os.environ.get("PYTHONPATH")
            if existing_pythonpath:
                paths.append(existing_pythonpath)
            return {"PYTHONPATH": os.pathsep.join(paths)}

    return {}


def _default_config_path():
    package_share = Path(get_package_share_directory("tello_defect_pipeline"))
    return str(package_share / "config" / "pipeline.yaml")


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    venv_env = _venv_pythonpath_env()

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=_default_config_path(), description="Path to the ROS parameter YAML file."),
        DeclareLaunchArgument("use_viewer", default_value="true", description="Start rqt_image_view for /defect_detections/image."),
        DeclareLaunchArgument("use_keyboard", default_value="false", description="Start keyboard teleoperation in this launch process. Running teleop separately is usually better for key focus."),
        Node(
            package="tello_defect_pipeline",
            executable="tello_bridge_node",
            name="tello_bridge_node",
            output="screen",
            additional_env=venv_env,
            parameters=[config_file],
        ),
        Node(
            package="tello_defect_pipeline",
            executable="defect_detector_node",
            name="defect_detector_node",
            output="screen",
            additional_env=venv_env,
            parameters=[config_file],
        ),
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="defect_image_view",
            output="screen",
            arguments=["/defect_detections/image"],
            condition=IfCondition(LaunchConfiguration("use_viewer")),
        ),
        Node(
            package="tello_defect_pipeline",
            executable="tello_keyboard_controller_node",
            name="tello_keyboard_controller_node",
            output="screen",
            emulate_tty=True,
            additional_env=venv_env,
            parameters=[config_file],
            condition=IfCondition(LaunchConfiguration("use_keyboard")),
        ),
    ])
