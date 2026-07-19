"""ROS 2 node for locating a wall target and annotating steel defects."""

from __future__ import annotations

from collections import deque
from contextlib import nullcontext
from pathlib import Path
import pickle
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import torch
    import torchvision.transforms as transforms
    import segmentation_models_pytorch as smp
except ImportError as exc:
    torch = None
    transforms = None
    smp = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


MODEL_INPUT_SIZE = (800, 256)
TARGET_ASPECT_MIN = 5.0
TARGET_ASPECT_MAX = 7.5
MASK_THRESHOLD = 0.5
OVERLAY_ALPHA = 0.45
DEFAULT_BENCHMARK_WINDOW = 60
DEFAULT_BENCHMARK_LOG_INTERVAL_SEC = 5.0
CLASS_COLORS = np.array(
    [
        (0, 0, 255),
        (0, 165, 255),
        (0, 255, 255),
        (255, 0, 0),
    ],
    dtype=np.uint8,
)


def _image_msg_to_bgr_frame(msg: Image) -> np.ndarray:
    """Convert bgr8/rgb8 ROS Image data into a BGR OpenCV frame."""
    if msg.encoding not in {"bgr8", "rgb8"}:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    expected_step = msg.width * 3
    if msg.step < expected_step:
        raise ValueError(
            f"Image step {msg.step} is smaller than expected {expected_step}."
        )

    image = np.frombuffer(msg.data, dtype=np.uint8)
    expected_size = msg.height * msg.step
    if image.size < expected_size:
        raise ValueError("Image data is shorter than height * step.")

    image = image[:expected_size].reshape((msg.height, msg.step))
    frame = image[:, :expected_step].reshape((msg.height, msg.width, 3))
    if msg.encoding == "rgb8":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame.copy()


def _bgr_frame_to_image_msg(frame: np.ndarray) -> Image:
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


def _default_model_path() -> str:
    """Return the installed model path, with a source-tree fallback."""
    try:
        package_share = Path(get_package_share_directory("tello_defect_pipeline"))
        return str(package_share / "models" / "model.pth")
    except PackageNotFoundError:
        return str(Path(__file__).resolve().parents[1] / "models" / "model.pth")


class DefectDetectorNode(Node):
    """Detect the Severstal target in camera frames and publish annotations."""

    def __init__(self) -> None:
        super().__init__("defect_detector_node")

        self.model = None
        self.device = None
        self.preprocess = None
        self.use_amp = False

        self.declare_parameter("model_path", _default_model_path())
        default_device = self._default_device_name()
        self.declare_parameter("device", default_device)
        self.declare_parameter("benchmark_enabled", True)
        self.declare_parameter(
            "benchmark_window",
            DEFAULT_BENCHMARK_WINDOW,
        )
        self.declare_parameter(
            "benchmark_log_interval_sec",
            DEFAULT_BENCHMARK_LOG_INTERVAL_SEC,
        )

        self.model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        device_name = self.get_parameter("device").get_parameter_value().string_value
        self.benchmark_enabled = (
            self.get_parameter("benchmark_enabled")
            .get_parameter_value()
            .bool_value
        )
        self.benchmark_window = max(
            1,
            self.get_parameter("benchmark_window")
            .get_parameter_value()
            .integer_value,
        )
        self.benchmark_log_interval_sec = max(
            0.5,
            self.get_parameter("benchmark_log_interval_sec")
            .get_parameter_value()
            .double_value,
        )
        self._latency_samples_ms: deque[float] = deque(
            maxlen=self.benchmark_window
        )
        self._inference_samples_ms: deque[float] = deque(
            maxlen=self.benchmark_window
        )
        self._publish_timestamps: deque[float] = deque(
            maxlen=self.benchmark_window
        )
        self._last_benchmark_log_time = time.perf_counter()
        self._last_inference_latency_ms: Optional[float] = None

        self._initialize_inference(device_name)

        self.image_subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )
        self.image_publisher = self.create_publisher(
            Image,
            "/defect_detections/image",
            10,
        )

    def _default_device_name(self) -> str:
        if torch is not None and torch.cuda.is_available():
            return "cuda"
        return "cpu"

    def _initialize_inference(self, device_name: str) -> None:
        """Load PyTorch dependencies and model weights if available."""
        if IMPORT_ERROR is not None:
            self.get_logger().error(
                f"PyTorch inference dependencies are unavailable: {IMPORT_ERROR}"
            )
            self.get_logger().warn("Publishing annotated frames without inference.")
            return

        try:
            requested_device = (
                device_name.strip().lower() or self._default_device_name()
            )
            if requested_device == "cuda" and not torch.cuda.is_available():
                self.get_logger().warn(
                    "CUDA was requested but is unavailable; falling back to CPU."
                )
                requested_device = "cpu"

            self.device = torch.device(requested_device)
            self.use_amp = self.device.type == "cuda"
            self.preprocess = transforms.Compose(
                [
                    transforms.ToTensor(),
                    transforms.Normalize(
                        mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225],
                    ),
                ]
            )

            self.model = smp.FPN(
                encoder_name="mit_b2",
                encoder_weights=None,
                in_channels=3,
                classes=4,
            )
            self.model.to(self.device)
            self._load_model_weights(Path(self.model_path))
            self.model.eval()

            self.get_logger().info(
                f"Defect model loaded from {self.model_path} on {self.device}."
            )
        except Exception as exc:  # noqa: BLE001 - keep image publishing alive.
            self.model = None
            self.get_logger().error(f"Failed to initialize defect model: {exc}")
            self.get_logger().warn("Publishing annotated frames without inference.")

    def _load_model_weights(self, model_path: Path) -> None:
        if not model_path.is_file():
            raise FileNotFoundError(f"Model weights not found: {model_path}")

        try:
            checkpoint = torch.load(
                model_path,
                map_location=self.device,
                weights_only=True,
            )
        except pickle.UnpicklingError:
            self.get_logger().warn(
                "Checkpoint requires legacy pickle loading; only use trusted model "
                "files because weights_only=False can execute arbitrary code."
            )
            checkpoint = torch.load(
                model_path,
                map_location=self.device,
                weights_only=False,
            )
        if isinstance(checkpoint, dict):
            state_dict = (
                checkpoint.get("state_dict")
                or checkpoint.get("model_state_dict")
                or checkpoint.get("model")
                or checkpoint
            )
        else:
            state_dict = checkpoint

        if not isinstance(state_dict, dict):
            raise TypeError("Model checkpoint does not contain a state dict.")

        cleaned_state_dict = {
            key.removeprefix("module.").removeprefix("model."): value
            for key, value in state_dict.items()
        }
        self.model.load_state_dict(cleaned_state_dict)

    def image_callback(self, msg: Image) -> None:
        """Annotate each camera frame, publish the result, and track runtime stats."""
        callback_start = time.perf_counter()
        self._last_inference_latency_ms = None

        try:
            frame = _image_msg_to_bgr_frame(msg)
        except ValueError as exc:
            self.get_logger().error(f"Failed to convert ROS image: {exc}")
            return

        try:
            annotated = self.process_frame(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to process frame: {exc}")
            annotated = frame.copy()
            cv2.putText(
                annotated,
                "Detector Error",
                (20, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        image_msg = _bgr_frame_to_image_msg(annotated)
        image_msg.header = msg.header
        self.image_publisher.publish(image_msg)

        if self.benchmark_enabled:
            callback_latency_ms = (time.perf_counter() - callback_start) * 1000.0
            self._record_benchmark_sample(callback_latency_ms)

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Find the target in a BGR frame and optionally run model inference."""
        annotated = frame.copy()
        target_box = self._find_target_box(frame)

        if target_box is None:
            cv2.putText(
                annotated,
                "Searching for Target...",
                (20, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            return annotated

        x, y, width, height = target_box
        crop = frame[y : y + height, x : x + width]
        status_text = "Target Locked"
        text_color = (0, 255, 0)

        if self.model is not None:
            try:
                colored_mask = self._predict_mask(crop)
            except Exception as exc:  # noqa: BLE001 - keep annotated video alive.
                self.get_logger().error(
                    f"Inference failed: {exc}",
                    throttle_duration_sec=2.0,
                )
            else:
                has_defect = bool(np.any(colored_mask))
                resized_mask = cv2.resize(
                    colored_mask,
                    (width, height),
                    interpolation=cv2.INTER_NEAREST,
                )
                blended_crop = cv2.addWeighted(
                    annotated[y : y + height, x : x + width],
                    1.0,
                    resized_mask,
                    OVERLAY_ALPHA,
                    0.0,
                )
                annotated[y : y + height, x : x + width] = blended_crop

                if has_defect:
                    status_text = "DEFECT DETECTED"
                    text_color = (0, 0, 255)
        else:
            status_text = "Target Locked - Model Unavailable"
            text_color = (0, 255, 255)

        cv2.rectangle(annotated, (x, y), (x + width, y + height), text_color, 2)
        label_y = max(30, y - 10)
        cv2.putText(
            annotated,
            status_text,
            (x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            text_color,
            2,
            cv2.LINE_AA,
        )
        return annotated

    def _find_target_box(
        self,
        frame: np.ndarray,
    ) -> Optional[tuple[int, int, int, int]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        _, thresholded = cv2.threshold(
            blurred,
            0,
            255,
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU,
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            thresholded,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None

        frame_area = frame.shape[0] * frame.shape[1]
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < frame_area * 0.02:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            if height == 0:
                continue

            aspect_ratio = width / float(height)
            if TARGET_ASPECT_MIN <= aspect_ratio <= TARGET_ASPECT_MAX:
                return (x, y, width, height)

        return None

    def _record_benchmark_sample(self, callback_latency_ms: float) -> None:
        now = time.perf_counter()
        self._latency_samples_ms.append(callback_latency_ms)
        self._publish_timestamps.append(now)
        if self._last_inference_latency_ms is not None:
            self._inference_samples_ms.append(self._last_inference_latency_ms)

        if now - self._last_benchmark_log_time < self.benchmark_log_interval_sec:
            return

        self._last_benchmark_log_time = now
        fps = self._rolling_fps()
        avg_latency = self._average(self._latency_samples_ms)
        max_latency = max(self._latency_samples_ms, default=0.0)
        avg_inference = self._average(self._inference_samples_ms)
        max_inference = max(self._inference_samples_ms, default=0.0)
        device_name = str(self.device) if self.device is not None else "none"

        self.get_logger().info(
            "Benchmark "
            f"device={device_name} "
            f"published_fps={fps:.2f} "
            f"avg_detection_latency_ms={avg_latency:.1f} "
            f"max_detection_latency_ms={max_latency:.1f} "
            f"avg_inference_latency_ms={avg_inference:.1f} "
            f"max_inference_latency_ms={max_inference:.1f} "
            f"samples={len(self._latency_samples_ms)}"
        )

    def _rolling_fps(self) -> float:
        if len(self._publish_timestamps) < 2:
            return 0.0
        elapsed = self._publish_timestamps[-1] - self._publish_timestamps[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self._publish_timestamps) - 1) / elapsed

    def _average(self, samples: deque[float]) -> float:
        if not samples:
            return 0.0
        return sum(samples) / len(samples)

    def _synchronize_device(self) -> None:
        if (
            torch is not None
            and self.device is not None
            and self.device.type == "cuda"
            and torch.cuda.is_available()
        ):
            torch.cuda.synchronize(self.device)

    def _predict_mask(self, crop_bgr: np.ndarray) -> np.ndarray:
        crop_resized = cv2.resize(
            crop_bgr,
            MODEL_INPUT_SIZE,
            interpolation=cv2.INTER_AREA,
        )
        crop_rgb = cv2.cvtColor(crop_resized, cv2.COLOR_BGR2RGB)
        image_tensor = self.preprocess(crop_rgb).unsqueeze(0).to(self.device)

        autocast_context = (
            torch.amp.autocast(device_type=self.device.type, enabled=self.use_amp)
            if hasattr(torch, "amp")
            else nullcontext()
        )
        self._synchronize_device()
        inference_start = time.perf_counter()
        with torch.no_grad(), autocast_context:
            logits = self.model(image_tensor)
            probabilities_tensor = torch.sigmoid(logits).squeeze(0).detach()
        self._synchronize_device()
        self._last_inference_latency_ms = (
            time.perf_counter() - inference_start
        ) * 1000.0

        probabilities = probabilities_tensor.cpu().numpy()
        masks = probabilities > MASK_THRESHOLD
        colored_mask = np.zeros(
            (MODEL_INPUT_SIZE[1], MODEL_INPUT_SIZE[0], 3),
            dtype=np.uint8,
        )
        for class_index, color in enumerate(CLASS_COLORS):
            colored_mask[masks[class_index]] = color

        return colored_mask


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DefectDetectorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
