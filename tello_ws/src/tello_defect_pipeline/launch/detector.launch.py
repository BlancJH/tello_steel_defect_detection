"""Launch the defect detector node with YAML configuration."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=_default_config_path(), description="Path to the ROS parameter YAML file."),
        Node(
            package="tello_defect_pipeline",
            executable="defect_detector_node",
            name="defect_detector_node",
            output="screen",
            additional_env=_venv_pythonpath_env(),
            parameters=[LaunchConfiguration("config_file")],
        ),
    ])
