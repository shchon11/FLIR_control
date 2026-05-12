#!/usr/bin/env python3
"""Interactive ROS 1 intrinsic calibration node for FLIR compressed images."""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CompressedImage


def load_yaml(path: str) -> dict:
    if not path:
        return {}
    yaml_path = Path(path)
    if not yaml_path.exists():
        return {}
    with yaml_path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def node_params_from_file(path: str, node_name: str) -> dict:
    payload = load_yaml(path)
    return dict(payload.get(node_name, {}).get("ros__parameters", {}))


def param(name: str, default: Any) -> Any:
    private_name = f"~{name}"
    return rospy.get_param(private_name, default)


def parse_cameras_file(path: str) -> list[dict]:
    payload = load_yaml(path)
    cameras = payload.get("flir_multicam", {}).get("ros__parameters", {}).get("cameras", [])
    if not isinstance(cameras, list):
        return []
    result = []
    for index, camera in enumerate(cameras):
        if not isinstance(camera, dict):
            continue
        item = dict(camera)
        item.setdefault("name", f"camera{index}")
        item.setdefault("namespace", item["name"])
        item.setdefault("frame_id", f"{item['name']}_optical_frame")
        result.append(item)
    return result


def select_camera(cameras: list[dict], camera_name: str, camera_serial: str) -> dict | None:
    if camera_serial:
        return next((camera for camera in cameras if str(camera.get("serial", "")) == camera_serial), None)
    if camera_name:
        return next((camera for camera in cameras if camera.get("name") == camera_name), None)
    return cameras[0] if cameras else None


def resolve_topic(topic: str, namespace: str) -> str:
    if topic.startswith("/"):
        return topic
    namespace = namespace.strip("/")
    return f"/{namespace}/{topic}" if namespace else topic


def format_float_list(values: list[float]) -> list[float]:
    return [float(f"{value:.10f}") for value in values]


class IntrinsicCalibrationNode:
    def __init__(self) -> None:
        params_file = rospy.get_param("~params_file", "")
        defaults = node_params_from_file(params_file, "flir_camera_calibration")

        self.input_topic = param("input_topic", defaults.get("input_topic", "/image_rgb/compressed"))
        self.annotated_output_topic = param(
            "annotated_output_topic",
            defaults.get("annotated_output_topic", "/calibration/image_annotated/compressed"),
        )
        self.output_yaml_path = param(
            "output_yaml_path",
            defaults.get("output_yaml_path", "calibration/flir_camera_info.yaml"),
        )
        self.camera_serial = str(param("camera_serial", defaults.get("camera_serial", "")))
        self.camera_name = str(param("camera_name", defaults.get("camera_name", "")))
        self.frame_id = str(param("frame_id", defaults.get("frame_id", "")))
        self.sample_image_dir = str(
            param("sample_image_dir", defaults.get("sample_image_dir", "calibration/captures"))
        )
        self.board_cols = int(param("board_cols", defaults.get("board_cols", 6)))
        self.board_rows = int(param("board_rows", defaults.get("board_rows", 5)))
        self.square_size_m = float(param("square_size_m", defaults.get("square_size_m", 0.08)))
        self.min_calibration_frames = int(
            param("min_calibration_frames", defaults.get("min_calibration_frames", 15))
        )
        self.display_window = bool(param("display_window", defaults.get("display_window", True)))
        self.window_name = str(param("window_name", defaults.get("window_name", "FLIR Calibration")))
        self.preview_scale = float(param("preview_scale", defaults.get("preview_scale", 1.0)))
        self.preview_max_width = int(param("preview_max_width", defaults.get("preview_max_width", 640)))
        self.preview_fast_check = bool(param("preview_fast_check", defaults.get("preview_fast_check", True)))
        self.annotated_jpeg_quality = int(
            param("annotated_jpeg_quality", defaults.get("annotated_jpeg_quality", 80))
        )
        self.queue_size = int(param("input_qos_depth", defaults.get("input_qos_depth", 1)))

        cameras_file = str(param("cameras_file", ""))
        selected_camera_name = str(param("select_camera_name", self.camera_name))
        if cameras_file:
            selected = select_camera(parse_cameras_file(cameras_file), selected_camera_name, self.camera_serial)
            if selected is None:
                raise RuntimeError(
                    f"Camera selection failed for name='{selected_camera_name}' serial='{self.camera_serial}'"
                )
            namespace = str(selected.get("namespace", selected.get("name", "")))
            self.camera_name = str(selected.get("name", self.camera_name or namespace))
            self.camera_serial = str(selected.get("serial", self.camera_serial))
            self.frame_id = str(selected.get("frame_id", self.frame_id or f"{self.camera_name}_optical_frame"))
            self.input_topic = resolve_topic(self.input_topic, namespace)
            self.annotated_output_topic = resolve_topic(self.annotated_output_topic, namespace)
            self.window_name = f"FLIR Calibration {self.camera_name}"

        if self.board_cols <= 0 or self.board_rows <= 0 or self.square_size_m <= 0:
            raise RuntimeError("board_cols, board_rows, and square_size_m must be positive.")

        self.board_size = (self.board_cols, self.board_rows)
        self.object_points_template = self.build_object_points()
        self.captured_image_points: list[np.ndarray] = []
        self.captured_object_points: list[np.ndarray] = []
        self.calibration_image_size: tuple[int, int] | None = None
        self.latest_msg: CompressedImage | None = None
        self.latest_annotated: np.ndarray | None = None
        self.latest_board_detected = False

        self.annotated_pub = rospy.Publisher(
            self.annotated_output_topic,
            CompressedImage,
            queue_size=1,
        )
        self.sub = rospy.Subscriber(
            self.input_topic,
            CompressedImage,
            self.on_image,
            queue_size=max(1, self.queue_size),
            buff_size=16 * 1024 * 1024,
        )

        if self.display_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        rospy.loginfo(
            "Intrinsic calibration listening on %s. space=capture c=calibrate r=reset q=quit",
            self.input_topic,
        )

    def build_object_points(self) -> np.ndarray:
        points = np.zeros((self.board_rows * self.board_cols, 3), np.float32)
        points[:, :2] = np.mgrid[0 : self.board_cols, 0 : self.board_rows].T.reshape(-1, 2)
        points *= float(self.square_size_m)
        return points

    def on_image(self, msg: CompressedImage) -> None:
        self.latest_msg = msg

    def decode(self, msg: CompressedImage) -> np.ndarray | None:
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(encoded, cv2.IMREAD_COLOR)

    def detect_chessboard(self, image: np.ndarray, fast_check: bool) -> tuple[bool, np.ndarray | None]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        if fast_check:
            flags |= cv2.CALIB_CB_FAST_CHECK
        detected, corners = cv2.findChessboardCorners(gray, self.board_size, flags)
        if not detected:
            return False, None
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        return True, corners

    def preview_image(self, image: np.ndarray) -> np.ndarray:
        if self.preview_max_width <= 0 or image.shape[1] <= self.preview_max_width:
            return image
        scale = float(self.preview_max_width) / float(image.shape[1])
        return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    def draw_overlay(self, image: np.ndarray, board_detected: bool) -> None:
        status = "Board detected" if board_detected else "Board not detected"
        color = (0, 220, 0) if board_detected else (0, 0, 255)
        lines = [
            (status, color),
            (f"Captured frames: {len(self.captured_image_points)}", (255, 255, 255)),
            ("space:capture  c:calib  r:reset  q:quit", (255, 255, 255)),
        ]
        for index, (text, line_color) in enumerate(lines):
            cv2.putText(
                image,
                text,
                (12, 24 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                text,
                (12, 24 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                line_color,
                1,
                cv2.LINE_AA,
            )

    def publish_annotated(self, header: Any, image: np.ndarray) -> None:
        if self.annotated_pub.get_num_connections() == 0:
            return
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), max(0, min(100, self.annotated_jpeg_quality))],
        )
        if not ok:
            return
        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.annotated_pub.publish(msg)

    def process_latest(self) -> None:
        msg = self.latest_msg
        if msg is None:
            return
        image = self.decode(msg)
        if image is None:
            rospy.logwarn_throttle(5.0, "Failed to decode compressed image from %s", self.input_topic)
            return
        preview = self.preview_image(image)
        detected, corners = self.detect_chessboard(preview, self.preview_fast_check)
        annotated = preview.copy()
        if detected and corners is not None:
            cv2.drawChessboardCorners(annotated, self.board_size, corners, detected)
        self.draw_overlay(annotated, detected)
        self.publish_annotated(msg.header, annotated)
        self.latest_annotated = annotated
        self.latest_board_detected = detected

    def save_capture_image(self, image: np.ndarray, capture_index: int) -> None:
        if not self.sample_image_dir:
            return
        target_dir = Path(self.sample_image_dir)
        if self.camera_serial:
            target_dir /= self.camera_serial
        target_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(target_dir / f"capture_{capture_index:03d}.jpg"), image)

    def capture_current_frame(self) -> None:
        msg = self.latest_msg
        if msg is None:
            rospy.logwarn("No image has been received yet.")
            return
        image = self.decode(msg)
        if image is None:
            rospy.logwarn("Failed to decode latest compressed image for capture.")
            return
        detected, corners = self.detect_chessboard(image, False)
        if not detected or corners is None:
            rospy.logwarn("Chessboard not detected in latest full-resolution frame. Capture skipped.")
            return
        image_size = (image.shape[1], image.shape[0])
        if self.calibration_image_size is None:
            self.calibration_image_size = image_size
        elif self.calibration_image_size != image_size:
            rospy.logwarn("Image size changed from %s to %s. Capture skipped.", self.calibration_image_size, image_size)
            return
        self.captured_image_points.append(corners)
        self.captured_object_points.append(self.object_points_template.copy())
        self.save_capture_image(image, len(self.captured_image_points))
        rospy.loginfo(
            "Captured calibration frame %d. Need at least %d frames before calibration.",
            len(self.captured_image_points),
            self.min_calibration_frames,
        )

    def reprojection_error(
        self,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
        rvecs: list[np.ndarray],
        tvecs: list[np.ndarray],
    ) -> float:
        total_error = 0.0
        total_points = 0
        for object_points, image_points, rvec, tvec in zip(
            self.captured_object_points,
            self.captured_image_points,
            rvecs,
            tvecs,
        ):
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
            error = cv2.norm(image_points, projected, cv2.NORM_L2)
            total_error += error * error
            total_points += len(projected)
        return math.sqrt(total_error / max(1, total_points))

    def write_calibration_yaml(
        self,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
        rms: float,
        mean_error: float,
    ) -> None:
        output_path = Path(self.output_yaml_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        k = format_float_list(camera_matrix.reshape(-1).tolist())
        d = format_float_list(distortion.reshape(-1).tolist())
        r = format_float_list(np.eye(3).reshape(-1).tolist())
        p = format_float_list(
            [
                camera_matrix[0, 0],
                0.0,
                camera_matrix[0, 2],
                0.0,
                0.0,
                camera_matrix[1, 1],
                camera_matrix[1, 2],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
        )

        payload = load_yaml(str(output_path))
        if self.camera_serial:
            payload.setdefault("version", 2)
            payload.setdefault("camera_info_by_serial", {})
            payload["camera_info_by_serial"][str(self.camera_serial)] = {
                "camera_name": self.camera_name or self.camera_serial,
                "frame_id": self.frame_id,
                "camera_info": {
                    "distortion_model": "plumb_bob",
                    "d": d,
                    "k": k,
                    "r": r,
                    "p": p,
                    "binning_x": 0,
                    "binning_y": 0,
                    "roi": {
                        "x_offset": 0,
                        "y_offset": 0,
                        "height": 0,
                        "width": 0,
                        "do_rectify": False,
                    },
                },
                "calibration": {
                    "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "image_width": self.calibration_image_size[0] if self.calibration_image_size else 0,
                    "image_height": self.calibration_image_size[1] if self.calibration_image_size else 0,
                    "board_cols": self.board_cols,
                    "board_rows": self.board_rows,
                    "square_size_m": float(self.square_size_m),
                    "captured_frames": len(self.captured_image_points),
                    "rms_reprojection_error": float(rms),
                    "mean_reprojection_error": float(mean_error),
                },
            }
        else:
            payload = {
                "flir_camera": {
                    "ros__parameters": {
                        "camera_info.distortion_model": "plumb_bob",
                        "camera_info.d": d,
                        "camera_info.k": k,
                        "camera_info.r": r,
                        "camera_info.p": p,
                    }
                },
                "calibration": {
                    "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "image_width": self.calibration_image_size[0] if self.calibration_image_size else 0,
                    "image_height": self.calibration_image_size[1] if self.calibration_image_size else 0,
                    "board_cols": self.board_cols,
                    "board_rows": self.board_rows,
                    "square_size_m": float(self.square_size_m),
                    "captured_frames": len(self.captured_image_points),
                    "rms_reprojection_error": float(rms),
                    "mean_reprojection_error": float(mean_error),
                },
            }

        with output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(payload, stream, sort_keys=False, default_flow_style=False)

    def calibrate_and_save(self) -> None:
        if len(self.captured_image_points) < max(1, self.min_calibration_frames):
            rospy.logwarn(
                "Need at least %d captured frames before calibration. Current count: %d",
                self.min_calibration_frames,
                len(self.captured_image_points),
            )
            return
        if self.calibration_image_size is None:
            rospy.logwarn("Calibration image size is invalid.")
            return
        rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            self.captured_object_points,
            self.captured_image_points,
            self.calibration_image_size,
            None,
            None,
        )
        mean_error = self.reprojection_error(camera_matrix, distortion, rvecs, tvecs)
        self.write_calibration_yaml(camera_matrix, distortion, rms, mean_error)
        rospy.loginfo(
            "Calibration finished with RMS %.6f and mean reprojection error %.6f. Saved to %s.",
            rms,
            mean_error,
            self.output_yaml_path,
        )

    def reset(self) -> None:
        self.captured_image_points.clear()
        self.captured_object_points.clear()
        self.calibration_image_size = None
        rospy.loginfo("Captured calibration frames cleared.")

    def run(self) -> None:
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.process_latest()
            if self.display_window and self.latest_annotated is not None:
                display = self.latest_annotated
                if abs(self.preview_scale - 1.0) > 1e-6:
                    display = cv2.resize(display, None, fx=self.preview_scale, fy=self.preview_scale)
                cv2.imshow(self.window_name, display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):
                    self.capture_current_frame()
                elif key in (ord("c"), ord("C")):
                    self.calibrate_and_save()
                elif key in (ord("r"), ord("R")):
                    self.reset()
                elif key in (ord("q"), ord("Q"), 27):
                    rospy.signal_shutdown("Calibration window requested shutdown.")
            rate.sleep()
        if self.display_window:
            cv2.destroyWindow(self.window_name)


def main() -> None:
    rospy.init_node("flir_camera_calibration")
    node = IntrinsicCalibrationNode()
    node.run()


if __name__ == "__main__":
    main()
