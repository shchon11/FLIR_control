#!/usr/bin/env python3
"""Interactive ROS 1 multicamera extrinsic calibration node."""

from __future__ import annotations

import math
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CameraInfo, CompressedImage


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
    return rospy.get_param(f"~{name}", default)


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
        item.setdefault("serial", "")
        result.append(item)
    return result


def strip_slash(value: str) -> str:
    return str(value or "").strip("/")


def namespaced_topic(namespace: str, topic: str) -> str:
    topic = strip_slash(topic)
    namespace = strip_slash(namespace)
    return f"/{namespace}/{topic}" if namespace else f"/{topic}"


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / norm


def quaternion_from_rotation(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion(np.array([x, y, z, w], dtype=np.float64))


def rotation_from_quaternion(q: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def format_float(value: float) -> float:
    return float(f"{float(value):.10f}")


@dataclass
class CameraRuntime:
    name: str
    namespace: str
    serial: str
    frame_id: str
    latest_msg: CompressedImage | None = None
    received_time: rospy.Time | None = None
    latest_bgr: np.ndarray | None = None
    annotated_bgr: np.ndarray | None = None
    corners: np.ndarray | None = None
    board_detected: bool = False
    camera_info: CameraInfo | None = None


@dataclass
class CameraPose:
    rotation_cam_board: np.ndarray
    translation_cam_board: np.ndarray
    reprojection_error: float


@dataclass
class Observation:
    camera_indices: list[int] = field(default_factory=list)
    poses: list[CameraPose] = field(default_factory=list)


@dataclass
class RelativePose:
    rotation: np.ndarray
    translation: np.ndarray
    reprojection_error: float = 0.0


class ExtrinsicCalibrationNode:
    def __init__(self) -> None:
        params_file = str(rospy.get_param("~params_file", ""))
        defaults = node_params_from_file(params_file, "flir_camera_extrinsic_calibration")

        self.cameras = self.load_cameras(defaults)
        if len(self.cameras) < 2:
            raise RuntimeError("At least two cameras are required for extrinsic calibration.")

        self.reference_camera = str(param("reference_camera", defaults.get("reference_camera", "camera0")))
        self.reference_frame = str(param("reference_frame", defaults.get("reference_frame", "flir_rig_frame")))
        self.image_topic = str(param("image_topic", defaults.get("image_topic", "image_rgb/compressed")))
        self.camera_info_topic = str(param("camera_info_topic", defaults.get("camera_info_topic", "camera_info")))
        self.output_yaml_path = str(
            param("output_yaml_path", defaults.get("output_yaml_path", "calibration/flir_camera_extrinsics.yaml"))
        )
        self.board_cols = int(param("board_cols", defaults.get("board_cols", 6)))
        self.board_rows = int(param("board_rows", defaults.get("board_rows", 5)))
        self.square_size_m = float(param("square_size_m", defaults.get("square_size_m", 0.08)))
        self.min_observations = int(param("min_observations", defaults.get("min_observations", 5)))
        self.max_frame_age_ms = int(param("max_frame_age_ms", defaults.get("max_frame_age_ms", 500)))
        self.require_all_cameras = bool(
            param("require_all_cameras_for_capture", defaults.get("require_all_cameras_for_capture", False))
        )
        self.display_window = bool(param("display_window", defaults.get("display_window", True)))
        self.window_name = str(param("window_name", defaults.get("window_name", "FLIR Extrinsic Calibration")))
        self.preview_max_width = int(param("preview_max_width", defaults.get("preview_max_width", 640)))
        self.preview_fast_check = bool(param("preview_fast_check", defaults.get("preview_fast_check", True)))
        self.queue_size = int(param("input_qos_depth", defaults.get("input_qos_depth", 1)))

        self.board_size = (self.board_cols, self.board_rows)
        self.object_points = self.build_object_points()
        self.reference_index = self.find_reference_index()
        self.observations: list[Observation] = []

        self.image_subs = []
        self.info_subs = []
        for index, camera in enumerate(self.cameras):
            image_topic = namespaced_topic(camera.namespace, self.image_topic)
            info_topic = namespaced_topic(camera.namespace, self.camera_info_topic)
            self.image_subs.append(
                rospy.Subscriber(
                    image_topic,
                    CompressedImage,
                    lambda msg, idx=index: self.on_image(idx, msg),
                    queue_size=max(1, self.queue_size),
                    buff_size=16 * 1024 * 1024,
                )
            )
            self.info_subs.append(
                rospy.Subscriber(
                    info_topic,
                    CameraInfo,
                    lambda msg, idx=index: self.on_camera_info(idx, msg),
                    queue_size=1,
                )
            )
            rospy.loginfo("Extrinsic calibration listening to %s and %s.", image_topic, info_topic)

        if self.display_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        rospy.loginfo("Extrinsic calibration ready. space=capture c=save r=reset q=quit")

    def load_cameras(self, defaults: dict) -> list[CameraRuntime]:
        cameras_file = str(param("cameras_file", ""))
        if cameras_file:
            entries = parse_cameras_file(cameras_file)
        else:
            namespaces = list(param("camera_namespaces", defaults.get("camera_namespaces", ["camera0", "camera1"])))
            names = list(param("camera_names", defaults.get("camera_names", namespaces)))
            serials = list(param("camera_serials", defaults.get("camera_serials", [""] * len(names))))
            frame_ids = list(
                param("camera_frame_ids", defaults.get("camera_frame_ids", [f"{name}_optical_frame" for name in names]))
            )
            entries = [
                {
                    "name": names[index],
                    "namespace": namespaces[index],
                    "serial": serials[index] if index < len(serials) else "",
                    "frame_id": frame_ids[index] if index < len(frame_ids) else f"{names[index]}_optical_frame",
                }
                for index in range(len(namespaces))
            ]
        return [
            CameraRuntime(
                name=str(entry.get("name", f"camera{index}")),
                namespace=str(entry.get("namespace", entry.get("name", f"camera{index}"))),
                serial=str(entry.get("serial", "")),
                frame_id=str(entry.get("frame_id", f"{entry.get('name', f'camera{index}')}_optical_frame")),
            )
            for index, entry in enumerate(entries)
        ]

    def build_object_points(self) -> np.ndarray:
        points = np.zeros((self.board_rows * self.board_cols, 3), np.float32)
        points[:, :2] = np.mgrid[0 : self.board_cols, 0 : self.board_rows].T.reshape(-1, 2)
        points *= float(self.square_size_m)
        return points

    def find_reference_index(self) -> int:
        for index, camera in enumerate(self.cameras):
            if self.reference_camera in {camera.name, camera.namespace, camera.serial}:
                return index
        raise RuntimeError(f"reference_camera '{self.reference_camera}' was not found.")

    def on_image(self, index: int, msg: CompressedImage) -> None:
        self.cameras[index].latest_msg = msg
        self.cameras[index].received_time = rospy.Time.now()

    def on_camera_info(self, index: int, msg: CameraInfo) -> None:
        self.cameras[index].camera_info = msg

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

    def process_camera(self, camera: CameraRuntime) -> None:
        if camera.latest_msg is None:
            return
        image = self.decode(camera.latest_msg)
        if image is None:
            return
        preview = self.preview_image(image)
        detected, preview_corners = self.detect_chessboard(preview, self.preview_fast_check)
        full_corners = None
        if detected and preview_corners is not None:
            full_corners = preview_corners.copy()
            if preview.shape[:2] != image.shape[:2]:
                full_corners[:, 0, 0] *= float(image.shape[1]) / float(preview.shape[1])
                full_corners[:, 0, 1] *= float(image.shape[0]) / float(preview.shape[0])

        annotated = preview.copy()
        if detected and preview_corners is not None:
            cv2.drawChessboardCorners(annotated, self.board_size, preview_corners, detected)
        color = (40, 220, 40) if detected else (40, 40, 230)
        text = f"{camera.name} {'board' if detected else 'no board'}"
        cv2.putText(annotated, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(annotated, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv2.LINE_AA)

        camera.latest_bgr = image
        camera.annotated_bgr = annotated
        camera.corners = full_corners
        camera.board_detected = detected

    def has_usable_camera_info(self, camera_info: CameraInfo | None) -> bool:
        return camera_info is not None and abs(float(camera_info.K[0])) > 1e-9 and abs(float(camera_info.K[4])) > 1e-9

    def camera_matrix(self, info: CameraInfo) -> np.ndarray:
        return np.array(info.K, dtype=np.float64).reshape(3, 3)

    def distortion(self, info: CameraInfo) -> np.ndarray:
        return np.array(info.D, dtype=np.float64).reshape(1, -1)

    def reprojection_error(self, image_points: np.ndarray, info: CameraInfo, rvec: np.ndarray, tvec: np.ndarray) -> float:
        projected, _ = cv2.projectPoints(self.object_points, rvec, tvec, self.camera_matrix(info), self.distortion(info))
        return float(cv2.norm(image_points, projected, cv2.NORM_L2) / math.sqrt(max(1, len(projected))))

    def solve_camera_pose(self, camera: CameraRuntime, corners: np.ndarray) -> CameraPose:
        if camera.camera_info is None:
            raise RuntimeError(f"{camera.name} has no camera_info")
        solved, rvec, tvec = cv2.solvePnP(
            self.object_points,
            corners,
            self.camera_matrix(camera.camera_info),
            self.distortion(camera.camera_info),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not solved:
            raise RuntimeError(f"solvePnP failed for {camera.name}")
        rotation, _ = cv2.Rodrigues(rvec)
        return CameraPose(
            rotation_cam_board=rotation,
            translation_cam_board=tvec.reshape(3),
            reprojection_error=self.reprojection_error(corners, camera.camera_info, rvec, tvec),
        )

    def capture_observation(self) -> None:
        now = rospy.Time.now()
        observation = Observation()
        for index, camera in enumerate(self.cameras):
            if camera.latest_msg is None:
                rospy.logwarn("Skipping %s: no image yet.", camera.name)
                continue
            if camera.received_time is None or (now - camera.received_time).to_sec() * 1000.0 > self.max_frame_age_ms:
                rospy.logwarn("Skipping %s: image is stale.", camera.name)
                continue
            if not self.has_usable_camera_info(camera.camera_info):
                rospy.logwarn("Skipping %s: no usable CameraInfo.", camera.name)
                continue
            image = self.decode(camera.latest_msg)
            if image is None:
                rospy.logwarn("Skipping %s: failed to decode latest image.", camera.name)
                continue
            detected, corners = self.detect_chessboard(image, False)
            if not detected or corners is None:
                rospy.logwarn("Skipping %s: chessboard was not detected.", camera.name)
                continue
            observation.camera_indices.append(index)
            observation.poses.append(self.solve_camera_pose(camera, corners))

        if self.require_all_cameras and len(observation.poses) != len(self.cameras):
            rospy.logwarn(
                "Not capturing: require_all_cameras_for_capture=true but only %d/%d cameras see the board.",
                len(observation.poses),
                len(self.cameras),
            )
            return
        if len(observation.poses) < 2:
            rospy.logwarn("Not capturing: need at least two cameras seeing the board. Got %d.", len(observation.poses))
            return

        self.observations.append(observation)
        names = ", ".join(self.cameras[index].name for index in observation.camera_indices)
        mean_error = sum(pose.reprojection_error for pose in observation.poses) / len(observation.poses)
        rospy.loginfo(
            "Captured pairwise graph observation %d with %s. Mean solvePnP reprojection error: %.4f px.",
            len(self.observations),
            names,
            mean_error,
        )

    def relative_pose_between(self, parent: CameraPose, child: CameraPose) -> RelativePose:
        rotation = parent.rotation_cam_board @ child.rotation_cam_board.T
        translation = parent.translation_cam_board - rotation @ child.translation_cam_board
        return RelativePose(rotation, translation, 0.5 * (parent.reprojection_error + child.reprojection_error))

    def invert_pose(self, pose: RelativePose) -> RelativePose:
        rotation = pose.rotation.T
        translation = -(rotation @ pose.translation)
        return RelativePose(rotation, translation, pose.reprojection_error)

    def compose_pose(self, parent_current: RelativePose, current_child: RelativePose) -> RelativePose:
        rotation = parent_current.rotation @ current_child.rotation
        translation = parent_current.rotation @ current_child.translation + parent_current.translation
        return RelativePose(rotation, translation, parent_current.reprojection_error + current_child.reprojection_error)

    def average_poses(self, poses: list[RelativePose]) -> RelativePose:
        translations = np.array([pose.translation for pose in poses], dtype=np.float64)
        quaternions = [quaternion_from_rotation(pose.rotation) for pose in poses]
        first = quaternions[0]
        aligned = [q if float(np.dot(q, first)) >= 0.0 else -q for q in quaternions]
        q_avg = normalize_quaternion(np.mean(np.array(aligned), axis=0))
        return RelativePose(
            rotation_from_quaternion(q_avg),
            np.mean(translations, axis=0),
            sum(pose.reprojection_error for pose in poses) / len(poses),
        )

    def build_graph(self) -> dict[int, list[tuple[int, RelativePose, int]]]:
        pair_observations: dict[tuple[int, int], list[RelativePose]] = defaultdict(list)
        for observation in self.observations:
            for first in range(len(observation.camera_indices)):
                for second in range(first + 1, len(observation.camera_indices)):
                    a = observation.camera_indices[first]
                    b = observation.camera_indices[second]
                    pose = self.relative_pose_between(observation.poses[first], observation.poses[second])
                    if a < b:
                        pair_observations[(a, b)].append(pose)
                    else:
                        pair_observations[(b, a)].append(self.invert_pose(pose))
        graph: dict[int, list[tuple[int, RelativePose, int]]] = defaultdict(list)
        for (parent, child), poses in pair_observations.items():
            average = self.average_poses(poses)
            graph[parent].append((child, average, len(poses)))
            graph[child].append((parent, self.invert_pose(average), len(poses)))
        return graph

    def solve_graph(self) -> dict[int, tuple[RelativePose, list[int], int, float]]:
        graph = self.build_graph()
        identity = RelativePose(np.eye(3), np.zeros(3), 0.0)
        solved: dict[int, tuple[RelativePose, list[int], int, float]] = {
            self.reference_index: (identity, [self.reference_index], 0, 0.0)
        }
        pending = deque([self.reference_index])
        while pending:
            current = pending.popleft()
            current_pose, current_path, current_min_obs, current_mean_error = solved[current]
            for child, edge_pose, count in graph.get(current, []):
                if child in solved:
                    continue
                composed = self.compose_pose(current_pose, edge_pose)
                path = current_path + [child]
                if current == self.reference_index:
                    min_obs = count
                    mean_error = edge_pose.reprojection_error
                else:
                    min_obs = min(current_min_obs, count)
                    previous_edges = max(1, len(current_path) - 1)
                    mean_error = (current_mean_error * previous_edges + edge_pose.reprojection_error) / (
                        previous_edges + 1
                    )
                solved[child] = (composed, path, min_obs, mean_error)
                pending.append(child)
        return solved

    def graph_path_text(self, path: list[int]) -> str:
        return " -> ".join(self.cameras[index].name for index in path)

    def write_extrinsics_yaml(self) -> None:
        if not self.observations:
            rospy.logwarn("Need at least one pairwise observation before saving extrinsics.")
            return
        solved = self.solve_graph()
        for index, camera in enumerate(self.cameras):
            if index not in solved:
                rospy.logwarn(
                    "Cannot save extrinsics: %s is not connected to reference camera %s.",
                    camera.name,
                    self.cameras[self.reference_index].name,
                )
                return
            _pose, path, min_obs, _mean_error = solved[index]
            if index != self.reference_index and min_obs < max(1, self.min_observations):
                rospy.logwarn(
                    "Cannot save extrinsics: path %s has only %d observations on weakest edge; need %d.",
                    self.graph_path_text(path),
                    min_obs,
                    self.min_observations,
                )
                return

        output_path = Path(self.output_yaml_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "reference_frame": self.reference_frame,
            "extrinsics_by_serial": {},
        }
        for index, camera in enumerate(self.cameras):
            pose, path, min_obs, mean_error = solved[index]
            q = quaternion_from_rotation(pose.rotation)
            serial_key = camera.serial or camera.name
            payload["extrinsics_by_serial"][str(serial_key)] = {
                "camera_name": camera.name,
                "parent_frame": self.reference_frame,
                "child_frame": camera.frame_id,
                "translation_xyz_m": [format_float(v) for v in pose.translation.tolist()],
                "rotation_xyzw": [format_float(v) for v in q.tolist()],
                "calibration": {
                    "method": "pairwise_graph_chessboard",
                    "reference_camera": self.cameras[self.reference_index].name,
                    "graph_path": self.graph_path_text(path),
                    "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "board_cols": self.board_cols,
                    "board_rows": self.board_rows,
                    "square_size_m": float(self.square_size_m),
                    "observations": len(self.observations),
                    "path_min_edge_observations": int(min_obs),
                    "mean_pair_reprojection_error": float(mean_error),
                },
            }
        with output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(payload, stream, sort_keys=False, default_flow_style=False)
        rospy.loginfo("Saved extrinsic calibration with %d observations to %s.", len(self.observations), output_path)

    def build_preview(self) -> np.ndarray:
        previews = []
        for camera in self.cameras:
            if camera.annotated_bgr is None:
                waiting = np.full((360, 480, 3), 30, dtype=np.uint8)
                cv2.putText(
                    waiting,
                    f"{camera.name} waiting",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (220, 220, 220),
                    1,
                    cv2.LINE_AA,
                )
                previews.append(waiting)
            else:
                scale = 360.0 / float(camera.annotated_bgr.shape[0])
                previews.append(cv2.resize(camera.annotated_bgr, None, fx=scale, fy=scale))
        max_height = max(preview.shape[0] for preview in previews)
        padded = []
        for preview in previews:
            if preview.shape[0] < max_height:
                pad = max_height - preview.shape[0]
                preview = cv2.copyMakeBorder(preview, 0, pad, 0, 0, cv2.BORDER_CONSTANT, value=(30, 30, 30))
            padded.append(preview)
        combined = cv2.hconcat(padded)
        text = f"observations {len(self.observations)}/{self.min_observations}  space:capture  c:save  r:reset  q:quit"
        cv2.putText(
            combined,
            text,
            (20, max(30, combined.shape[0] - 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            combined,
            text,
            (20, max(30, combined.shape[0] - 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return combined

    def reset(self) -> None:
        self.observations.clear()
        rospy.loginfo("Extrinsic observations cleared.")

    def run(self) -> None:
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            for camera in self.cameras:
                self.process_camera(camera)
            if self.display_window:
                preview = self.build_preview()
                cv2.imshow(self.window_name, preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):
                    self.capture_observation()
                elif key in (ord("c"), ord("C")):
                    self.write_extrinsics_yaml()
                elif key in (ord("r"), ord("R")):
                    self.reset()
                elif key in (ord("q"), ord("Q"), 27):
                    rospy.signal_shutdown("Extrinsic calibration window requested shutdown.")
            rate.sleep()
        if self.display_window:
            cv2.destroyWindow(self.window_name)


def main() -> None:
    rospy.init_node("flir_camera_extrinsic_calibration")
    node = ExtrinsicCalibrationNode()
    node.run()


if __name__ == "__main__":
    main()
