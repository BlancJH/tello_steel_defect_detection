import numpy as np
import pytest

from tello_defect_pipeline import defect_detector_node as detector_module
from sensor_msgs.msg import Image

from tello_defect_pipeline.defect_detector_node import (
    DefectDetectorNode,
    _bgr_frame_to_image_msg,
    _image_msg_to_bgr_frame,
)


def test_bgr_frame_to_image_msg_round_trip_bgr8():
    frame = np.array(
        [
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]],
        ],
        dtype=np.uint8,
    )

    msg = _bgr_frame_to_image_msg(frame)
    restored = _image_msg_to_bgr_frame(msg)

    assert msg.height == 2
    assert msg.width == 2
    assert msg.encoding == "bgr8"
    assert msg.step == 6
    np.testing.assert_array_equal(restored, frame)


def test_image_msg_to_bgr_frame_converts_rgb8_to_bgr():
    msg = Image()
    msg.height = 1
    msg.width = 2
    msg.encoding = "rgb8"
    msg.step = 6
    msg.data = bytes([10, 20, 30, 40, 50, 60])

    frame = _image_msg_to_bgr_frame(msg)

    expected = np.array([[[30, 20, 10], [60, 50, 40]]], dtype=np.uint8)
    np.testing.assert_array_equal(frame, expected)


def test_image_msg_to_bgr_frame_rejects_unsupported_encoding():
    msg = Image()
    msg.height = 1
    msg.width = 1
    msg.encoding = "mono8"
    msg.step = 1
    msg.data = bytes([0])

    with pytest.raises(ValueError, match="Unsupported image encoding"):
        _image_msg_to_bgr_frame(msg)


def test_image_msg_to_bgr_frame_rejects_short_step():
    msg = Image()
    msg.height = 1
    msg.width = 2
    msg.encoding = "bgr8"
    msg.step = 5
    msg.data = bytes([0] * 5)

    with pytest.raises(ValueError, match="smaller than expected"):
        _image_msg_to_bgr_frame(msg)


def test_find_target_box_accepts_expected_aspect_ratio():
    detector = DefectDetectorNode.__new__(DefectDetectorNode)
    detector.gaussian_kernel_size = 7
    detector.morph_kernel_size = (7, 7)
    detector.min_target_area_ratio = 0.02
    detector.target_aspect_min = 5.0
    detector.target_aspect_max = 7.5
    frame = np.full((240, 640, 3), 255, dtype=np.uint8)
    frame[80:160, 80:560] = 0

    box = DefectDetectorNode._find_target_box(detector, frame)

    assert box is not None
    x, y, width, height = box
    assert x <= 85
    assert y <= 85
    assert width >= 470
    assert height >= 70


def test_find_target_box_rejects_wrong_aspect_ratio():
    detector = DefectDetectorNode.__new__(DefectDetectorNode)
    detector.gaussian_kernel_size = 7
    detector.morph_kernel_size = (7, 7)
    detector.min_target_area_ratio = 0.02
    detector.target_aspect_min = 5.0
    detector.target_aspect_max = 7.5
    frame = np.full((240, 640, 3), 255, dtype=np.uint8)
    frame[60:180, 220:420] = 0

    assert DefectDetectorNode._find_target_box(detector, frame) is None


def test_parse_class_colors_requires_triplet_per_class():
    detector = DefectDetectorNode.__new__(DefectDetectorNode)
    detector.defect_classes = 2

    colors = DefectDetectorNode._parse_class_colors(
        detector,
        [0, 0, 255, 0, 255, 0],
    )

    assert colors.shape == (2, 3)

    with pytest.raises(ValueError, match="3 values for each defect class"):
        DefectDetectorNode._parse_class_colors(detector, [0, 0, 255])


def test_odd_kernel_size_normalizes_even_and_small_values():
    detector = DefectDetectorNode.__new__(DefectDetectorNode)

    assert DefectDetectorNode._odd_kernel_size(detector, 6) == 7
    assert DefectDetectorNode._odd_kernel_size(detector, 7) == 7
    assert DefectDetectorNode._odd_kernel_size(detector, 0) == 1


def test_callback_latency_synchronizes_before_stopping_timer(monkeypatch):
    detector = DefectDetectorNode.__new__(DefectDetectorNode)
    calls = []

    def synchronize_device():
        calls.append("synchronize")

    def perf_counter():
        calls.append("timer")
        return 10.012

    detector._synchronize_device = synchronize_device
    monkeypatch.setattr(detector_module.time, "perf_counter", perf_counter)

    latency_ms = DefectDetectorNode._callback_latency_ms(detector, 10.0)

    assert calls == ["synchronize", "timer"]
    assert latency_ms == pytest.approx(12.0)


def test_benchmark_average_and_rolling_fps():
    detector = DefectDetectorNode.__new__(DefectDetectorNode)
    detector._publish_timestamps = [10.0, 10.5, 11.0]

    assert DefectDetectorNode._average(detector, [1.0, 2.0, 3.0]) == 2.0
    assert DefectDetectorNode._average(detector, []) == 0.0
    assert DefectDetectorNode._rolling_fps(detector) == 2.0
