"""Launch the defect detector node with benchmark parameters."""

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


def _default_model_path():
    package_share = Path(get_package_share_directory("tello_defect_pipeline"))
    return str(package_share / "models" / "model.pth")


def generate_launch_description():
    venv_env = _venv_pythonpath_env()

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=_default_model_path(), description="Absolute path to the PyTorch model checkpoint."),
        DeclareLaunchArgument("device", default_value="", description="Inference device: cuda, cpu, or empty for automatic selection."),
        DeclareLaunchArgument("benchmark_enabled", default_value="true", description="Enable rolling FPS and latency benchmark logs."),
        DeclareLaunchArgument("benchmark_window", default_value="60", description="Number of frames used for rolling benchmark statistics."),
        DeclareLaunchArgument("benchmark_log_interval_sec", default_value="5.0", description="Seconds between benchmark log messages."),
        Node(
            package="tello_defect_pipeline",
            executable="defect_detector_node",
            name="defect_detector_node",
            output="screen",
            additional_env=venv_env,
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "device": LaunchConfiguration("device"),
                "benchmark_enabled": LaunchConfiguration("benchmark_enabled"),
                "benchmark_window": LaunchConfiguration("benchmark_window"),
                "benchmark_log_interval_sec": LaunchConfiguration("benchmark_log_interval_sec"),
            }],
        ),
    ])
