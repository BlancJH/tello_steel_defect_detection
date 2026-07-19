from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "pipeline.yaml"


def test_pipeline_yaml_has_required_node_sections():
    config = yaml.safe_load(CONFIG_PATH.read_text())

    assert "tello_bridge_node" in config
    assert "defect_detector_node" in config
    assert "tello_keyboard_controller_node" in config

    for node_name, node_config in config.items():
        assert "ros__parameters" in node_config, node_name


def test_pipeline_yaml_defines_core_topic_contract():
    config = yaml.safe_load(CONFIG_PATH.read_text())

    bridge = config["tello_bridge_node"]["ros__parameters"]
    detector = config["defect_detector_node"]["ros__parameters"]
    keyboard = config["tello_keyboard_controller_node"]["ros__parameters"]

    assert bridge["image_topic"] == detector["input_image_topic"]
    assert bridge["cmd_vel_topic"] == keyboard["cmd_vel_topic"]
    assert bridge["takeoff_topic"] == keyboard["takeoff_topic"]
    assert bridge["land_topic"] == keyboard["land_topic"]


def test_detector_color_config_matches_class_count():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    detector = config["defect_detector_node"]["ros__parameters"]

    assert len(detector["class_colors_bgr"]) == detector["defect_classes"] * 3


def test_launch_files_exist():
    launch_dir = PACKAGE_ROOT / "launch"

    assert (launch_dir / "live_pipeline.launch.py").is_file()
    assert (launch_dir / "detector.launch.py").is_file()
    assert (launch_dir / "teleop.launch.py").is_file()
    assert (launch_dir / "visualization.launch.py").is_file()
