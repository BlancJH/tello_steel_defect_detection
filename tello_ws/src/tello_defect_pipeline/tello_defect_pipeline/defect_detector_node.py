"""ROS 2 node for locating a wall target and annotating steel defects."""

from __future__ import annotations

from contextlib import nullcontext
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from cv_bridge import CvBridge, CvBridgeError
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
TARGET_ASPECT_MIN = 2.0
TARGET_ASPECT_MAX = 4.5
MASK_THRESHOLD = 0.5
OVERLAY_ALPHA = 0.45
CLASS_COLORS = np.array(
    [
        (0, 0, 255),
        (0, 165, 255),
        (0, 255, 255),
        (255, 0, 0),
    ],
    dtype=np.uint8,
)


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

        self.bridge = CvBridge()
        self.model = None
        self.device = None
        self.preprocess = None
        self.use_amp = False

        self.declare_parameter("model_path", _default_model_path())
        default_device = self._default_device_name()
        self.declare_parameter("device", default_device)

        self.model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        device_name = self.get_parameter("device").get_parameter_value().string_value

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

        checkpoint = torch.load(model_path, map_location=self.device)
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
        """Annotate each camera frame and publish the result."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
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

        try:
            image_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert annotated image: {exc}")
            return

        image_msg.header = msg.header
        self.image_publisher.publish(image_msg)

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
        with torch.no_grad(), autocast_context:
            logits = self.model(image_tensor)
            probabilities = (
                torch.sigmoid(logits).squeeze(0).detach().cpu().numpy()
            )

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
