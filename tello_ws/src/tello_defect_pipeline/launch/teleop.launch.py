"""Launch keyboard teleoperation for the Tello pipeline."""

from pathlib import Path
import os

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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("movement_timeout", default_value="0.25", description="Seconds before movement command returns to hover after key release."),
        Node(
            package="tello_defect_pipeline",
            executable="tello_keyboard_controller_node",
            name="tello_keyboard_controller_node",
            output="screen",
            emulate_tty=True,
            additional_env=_venv_pythonpath_env(),
            parameters=[{
                "movement_timeout": LaunchConfiguration("movement_timeout"),
            }],
        ),
    ])
