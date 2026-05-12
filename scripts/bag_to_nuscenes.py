#!/usr/bin/env python3
"""Convert FLIR ROS camera bags into an image-only nuScenes dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sqlite3
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

try:
    import rosbag
except ImportError:  # pragma: no cover - depends on sourced ROS 1 environment.
    rosbag = None

try:
    from rclpy.serialization import deserialize_message as deserialize_ros2_message
except ImportError:  # pragma: no cover - only needed for ROS 2 sqlite3 bags.
    deserialize_ros2_message = None


SUPPORTED_IMAGE_TYPES = {
    "sensor_msgs/CompressedImage",
    "sensor_msgs/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/Image",
}

COMPRESSED_IMAGE_TYPES = {
    "sensor_msgs/CompressedImage",
    "sensor_msgs/msg/CompressedImage",
}

RAW_IMAGE_TYPES = {
    "sensor_msgs/Image",
    "sensor_msgs/msg/Image",
}

CAMERA_INFO_TYPES = {
    "sensor_msgs/CameraInfo",
    "sensor_msgs/msg/CameraInfo",
}


@dataclass(frozen=True)
class TopicRecord:
    topic_id: int
    name: str
    msg_type: str


@dataclass(frozen=True)
class BagSource:
    path: Path
    storage: str
    db3_files: list[Path]
    ros1_bag_files: list[Path]

    @property
    def output_name(self) -> str:
        if self.storage == "ros1" and self.path.is_file():
            return self.path.stem
        return self.path.name


@dataclass
class CameraInfoRecord:
    width: int
    height: int
    distortion_model: str
    d: list[float]
    k: list[float]
    r: list[float]
    p: list[float]

    @property
    def intrinsic_3x3(self) -> list[list[float]]:
        return [
            self.k[0:3],
            self.k[3:6],
            self.k[6:9],
        ]


@dataclass
class ImageRecord:
    stream_id: str
    camera_id: str
    stream_kind: str
    channel: str
    topic: str
    msg_type: str
    timestamp_ns: int
    bag_timestamp_ns: int
    data: Any
    format: str
    width: int | None = None
    height: int | None = None


@dataclass
class ConversionSummary:
    bag: Path
    output: Path
    version: str
    scene_name: str
    selected_topics: list[str]
    channel_by_stream: dict[str, str]
    camera_by_stream: dict[str, str]
    frames_read_by_stream: dict[str, int]
    frames_written_by_channel: dict[str, int]
    samples_written: int
    sync_tolerance_ms: float
    undistorted_export: bool
    preserve_bayer_raw: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert ROS camera bags under FLIR_control/bags "
            "to image-only nuScenes layout."
        )
    )
    parser.add_argument(
        "bag",
        nargs="?",
        help=(
            "ROS 1 .bag, ROS 2 bag directory, or ROS 2 .db3 file. Defaults to "
            "the newest bag under ./bags."
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output dataset root. Defaults to ./nuscenes_export/<bag_name>.",
    )
    parser.add_argument(
        "--version",
        default="v1.0-mini",
        help="nuScenes table directory name to create. Default: v1.0-mini.",
    )
    parser.add_argument(
        "--image-topic",
        action="append",
        default=[],
        help=(
            "Image topic to export. Can be repeated. Defaults to undistorted "
            "image topics when present, otherwise RGB image topics that will be undistorted on export."
        ),
    )
    parser.add_argument(
        "--raw-bayer",
        action="store_true",
        help=(
            "Export raw sensor_msgs/Image topics only. Bayer encodings are colorized as "
            "the original sensor mosaic without demosaicing or undistortion."
        ),
    )
    parser.add_argument(
        "--raw-and-rgb",
        action="store_true",
        help=(
            "Export both /image_raw and /image_rgb topics per camera. Raw Bayer images are "
            "colorized as the original sensor mosaic, while RGB images stay RGB/BGR."
        ),
    )
    parser.add_argument(
        "--no-undistort",
        action="store_true",
        help="Write source image topics as-is instead of exporting undistorted images.",
    )
    parser.add_argument(
        "--camera-channel",
        action="append",
        default=[],
        metavar="CAMERA=CHANNEL",
        help=(
            "Map a ROS camera namespace to a nuScenes channel, e.g. "
            "camera0=CAM_FRONT. Can be repeated. Defaults to CAM_<CAMERA>."
        ),
    )
    parser.add_argument(
        "--camera-info-yaml",
        default="calibration/flir_camera_info.yaml",
        help="Fallback intrinsic YAML. Default: calibration/flir_camera_info.yaml.",
    )
    parser.add_argument(
        "--extrinsics-yaml",
        default="calibration/flir_camera_extrinsics.yaml",
        help="Optional rig extrinsics YAML. Default: calibration/flir_camera_extrinsics.yaml.",
    )
    parser.add_argument(
        "--sync-tolerance-ms",
        type=float,
        default=50.0,
        help="Max timestamp difference for grouping cameras into one sample. Default: 50.",
    )
    parser.add_argument(
        "--scene-name",
        help="Scene name written into scene.json. Defaults to the bag directory name.",
    )
    parser.add_argument(
        "--description",
        default="FLIR multicamera ROS bag converted to image-only nuScenes format.",
        help="Scene description.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Export at most N synchronized samples. Useful for quick validation.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete an existing output directory before exporting.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Inspect bag and print the planned export without writing files.",
    )
    return parser.parse_args()


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[1]


def require_ros1_bag_module() -> Any:
    if rosbag is None:
        raise RuntimeError(
            "ROS 1 bag support requires the rosbag Python module. "
            "Run `source /opt/ros/noetic/setup.bash` or "
            "`source scripts/setup_flir_env.bash` before this script."
        )
    return rosbag


def require_ros2_deserializer() -> Any:
    if deserialize_ros2_message is None:
        raise RuntimeError(
            "ROS 2 sqlite3 bag support requires rclpy. This Noetic workspace "
            "can convert ROS 1 .bag files directly; pass a .bag file or record "
            "with rosbag."
        )
    return deserialize_ros2_message


def is_compressed_image_type(msg_type: str) -> bool:
    return msg_type in COMPRESSED_IMAGE_TYPES


def is_raw_image_type(msg_type: str) -> bool:
    return msg_type in RAW_IMAGE_TYPES


def is_camera_info_type(msg_type: str) -> bool:
    return msg_type in CAMERA_INFO_TYPES


def is_supported_image_type(msg_type: str) -> bool:
    return msg_type in SUPPORTED_IMAGE_TYPES


def is_ros_image_message(value: Any) -> bool:
    return all(hasattr(value, field) for field in ("height", "width", "encoding", "step", "data"))


def latest_bag_path(root: Path) -> Path:
    bags_root = root / "bags"
    if not bags_root.exists():
        raise FileNotFoundError(f"Bag directory does not exist: {bags_root}")

    ros2_dirs = [
        path
        for path in bags_root.iterdir()
        if path.is_dir() and (path / "metadata.yaml").exists()
    ]
    ros1_files = [path for path in bags_root.iterdir() if path.is_file() and path.suffix == ".bag"]
    candidates = ros2_dirs + ros1_files
    if not candidates:
        raise FileNotFoundError(f"No ROS .bag files or ROS 2 bag directories found under {bags_root}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def resolve_bag(input_path: str | None, root: Path) -> BagSource:
    bag_path = Path(input_path).expanduser() if input_path else latest_bag_path(root)
    if not bag_path.is_absolute():
        bag_path = (root / bag_path).resolve()

    if bag_path.is_file() and bag_path.suffix == ".bag":
        return BagSource(
            path=bag_path,
            storage="ros1",
            db3_files=[],
            ros1_bag_files=[bag_path],
        )
    if bag_path.is_file() and bag_path.suffix == ".db3":
        return BagSource(
            path=bag_path.parent,
            storage="ros2",
            db3_files=[bag_path],
            ros1_bag_files=[],
        )
    if bag_path.is_dir():
        db3_files = sorted(bag_path.glob("*.db3"))
        if db3_files:
            return BagSource(
                path=bag_path,
                storage="ros2",
                db3_files=db3_files,
                ros1_bag_files=[],
            )
        ros1_bag_files = sorted(bag_path.glob("*.bag"))
        if ros1_bag_files:
            return BagSource(
                path=bag_path,
                storage="ros1",
                db3_files=[],
                ros1_bag_files=ros1_bag_files,
            )

    raise FileNotFoundError(f"Could not find ROS .bag or ROS 2 .db3 files for bag path: {bag_path}")


def token_for(*parts: object) -> str:
    joined = "::".join(str(part) for part in parts)
    return hashlib.md5(joined.encode("utf-8")).hexdigest()


def ns_to_us(timestamp_ns: int) -> int:
    return timestamp_ns // 1000


def stamp_to_ns(stamp: Any, fallback_ns: int) -> int:
    if hasattr(stamp, "to_nsec"):
        value = int(stamp.to_nsec())
        return value if value > 0 else int(fallback_ns)

    sec = getattr(stamp, "sec", getattr(stamp, "secs", 0))
    nanosec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", 0))
    value = int(sec) * 1_000_000_000 + int(nanosec)
    return value if value > 0 else int(fallback_ns)


def load_topics_ros2(db3_files: list[Path]) -> dict[int, TopicRecord]:
    topics: dict[int, TopicRecord] = {}
    for db3_file in db3_files:
        with sqlite3.connect(db3_file) as connection:
            for topic_id, name, msg_type in connection.execute(
                "SELECT id, name, type FROM topics ORDER BY id"
            ):
                topics[int(topic_id)] = TopicRecord(int(topic_id), str(name), str(msg_type))
    return topics


def load_topics_ros1(bag_files: list[Path]) -> dict[int, TopicRecord]:
    rosbag_module = require_ros1_bag_module()
    topics_by_name: dict[str, TopicRecord] = {}
    for bag_file in bag_files:
        with rosbag_module.Bag(str(bag_file), "r") as bag:
            _types, topic_infos = bag.get_type_and_topic_info()
            for name, info in sorted(topic_infos.items()):
                if name not in topics_by_name:
                    topics_by_name[name] = TopicRecord(
                        len(topics_by_name) + 1,
                        str(name),
                        str(info.msg_type),
                    )
    return {topic.topic_id: topic for topic in topics_by_name.values()}


def load_topics(source: BagSource) -> dict[int, TopicRecord]:
    if source.storage == "ros1":
        return load_topics_ros1(source.ros1_bag_files)
    return load_topics_ros2(source.db3_files)


def camera_id_from_topic(topic: str) -> str:
    parts = [part for part in topic.split("/") if part]
    if not parts:
        return "camera"

    for part in parts:
        if re.fullmatch(r"camera[\w-]*", part, flags=re.IGNORECASE):
            return part
    return parts[0]


def is_undistorted_topic(topic: str) -> bool:
    return "undistort" in topic.lower()


def is_raw_image_topic(topic: TopicRecord | str) -> bool:
    name = topic.name if isinstance(topic, TopicRecord) else topic
    return bool(re.search(r"(^|/)image_raw($|/)", name))


def is_rgb_image_topic(topic: TopicRecord | str) -> bool:
    name = topic.name if isinstance(topic, TopicRecord) else topic
    return bool(re.search(r"(^|/)image_rgb($|/)", name)) or "rgb" in name.lower()


def stream_kind_from_topic(topic: str) -> str:
    if is_raw_image_topic(topic):
        return "raw"
    if is_rgb_image_topic(topic):
        return "rgb"
    if is_undistorted_topic(topic):
        return "undistorted"
    return "image"


def stream_id_for(camera_id: str, kind: str, needs_suffix: bool) -> str:
    return f"{camera_id}_{kind}" if needs_suffix else camera_id


def default_channel(camera_id: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9]+", "_", camera_id).strip("_").upper()
    return f"CAM_{normalized or 'CAMERA'}"


def default_stream_channel(
    camera_id: str,
    stream_id: str,
    kind: str,
    needs_suffix: bool,
    channel_overrides: dict[str, str],
) -> str:
    if stream_id in channel_overrides:
        return channel_overrides[stream_id]

    base_channel = channel_overrides.get(camera_id, default_channel(camera_id))
    if needs_suffix:
        return f"{base_channel}_{kind.upper()}"
    return base_channel


def parse_channel_overrides(values: list[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"Expected --camera-channel value in CAMERA=CHANNEL form: {value}")
        camera_id, channel = value.split("=", 1)
        camera_id = camera_id.strip()
        channel = channel.strip()
        if not camera_id or not channel:
            raise ValueError(f"Expected non-empty CAMERA=CHANNEL mapping: {value}")
        overrides[camera_id] = channel
    return overrides


def prefer_undistorted_topics(candidates: list[TopicRecord]) -> list[TopicRecord]:
    selected: list[TopicRecord] = []
    by_camera: dict[str, list[TopicRecord]] = {}
    for topic in candidates:
        by_camera.setdefault(camera_id_from_topic(topic.name), []).append(topic)

    for camera_topics in by_camera.values():
        undistorted = [topic for topic in camera_topics if is_undistorted_topic(topic.name)]
        selected.extend(undistorted or camera_topics)
    return selected


def choose_image_topics(
    topics: dict[int, TopicRecord],
    requested_topics: list[str],
    raw_bayer: bool,
    raw_and_rgb: bool,
) -> list[TopicRecord]:
    if requested_topics:
        requested = set(requested_topics)
        selected = [topic for topic in topics.values() if topic.name in requested]
        missing = sorted(requested - {topic.name for topic in selected})
        if missing:
            raise ValueError(f"Requested image topics not found in bag: {', '.join(missing)}")
    elif raw_and_rgb:
        raw_topics = [
            topic
            for topic in topics.values()
            if is_raw_image_type(topic.msg_type) and is_raw_image_topic(topic)
        ]
        rgb_topics = [
            topic
            for topic in topics.values()
            if is_supported_image_type(topic.msg_type) and is_rgb_image_topic(topic)
        ]
        if not raw_topics:
            raise ValueError("No /image_raw sensor_msgs/Image topics found for --raw-and-rgb.")
        if not rgb_topics:
            raise ValueError("No /image_rgb topics found for --raw-and-rgb.")
        selected = raw_topics + prefer_undistorted_topics(rgb_topics)
    elif raw_bayer:
        selected = [
            topic
            for topic in topics.values()
            if is_raw_image_type(topic.msg_type) and is_raw_image_topic(topic)
        ]
        if not selected:
            raise ValueError("No /image_raw sensor_msgs/Image topics found for --raw-bayer.")
    else:
        image_topics = [
            topic
            for topic in topics.values()
            if is_supported_image_type(topic.msg_type) and "/image" in topic.name
        ]
        undistorted_topics = [topic for topic in image_topics if is_undistorted_topic(topic.name)]
        rgb_topics = [topic for topic in image_topics if is_rgb_image_topic(topic)]
        raw_topics = [topic for topic in image_topics if is_raw_image_topic(topic)]
        selected = undistorted_topics or rgb_topics or raw_topics or image_topics

    selected = sorted(selected, key=lambda topic: topic.name)
    if not selected:
        raise ValueError("No image topics found. Use --image-topic to select topics explicitly.")
    unsupported = [topic for topic in selected if not is_supported_image_type(topic.msg_type)]
    if unsupported:
        details = ", ".join(f"{topic.name} ({topic.msg_type})" for topic in unsupported)
        raise ValueError(f"Unsupported image topic type(s): {details}")
    return selected


def camera_info_record_from_msg(msg: CameraInfo) -> CameraInfoRecord:
    return CameraInfoRecord(
        width=int(msg.width),
        height=int(msg.height),
        distortion_model=str(msg.distortion_model),
        d=[float(value) for value in getattr(msg, "d", getattr(msg, "D", []))],
        k=[float(value) for value in getattr(msg, "k", getattr(msg, "K", []))],
        r=[float(value) for value in getattr(msg, "r", getattr(msg, "R", []))],
        p=[float(value) for value in getattr(msg, "p", getattr(msg, "P", []))],
    )


def load_camera_info_from_ros2_bag(
    db3_files: list[Path],
    topics: dict[int, TopicRecord],
) -> dict[str, CameraInfoRecord]:
    info_topic_ids = {
        topic.topic_id: camera_id_from_topic(topic.name)
        for topic in topics.values()
        if is_camera_info_type(topic.msg_type)
    }
    camera_info: dict[str, CameraInfoRecord] = {}
    if not info_topic_ids:
        return camera_info

    for db3_file in db3_files:
        with sqlite3.connect(db3_file) as connection:
            for topic_id, data, _timestamp_ns in connection.execute(
                "SELECT topic_id, data, timestamp FROM messages ORDER BY timestamp"
            ):
                camera_id = info_topic_ids.get(int(topic_id))
                if camera_id is None or camera_id in camera_info:
                    continue
                msg = require_ros2_deserializer()(data, CameraInfo)
                camera_info[camera_id] = camera_info_record_from_msg(msg)
                if len(camera_info) == len(set(info_topic_ids.values())):
                    return camera_info
    return camera_info


def load_camera_info_from_ros1_bag(
    bag_files: list[Path],
    topics: dict[int, TopicRecord],
) -> dict[str, CameraInfoRecord]:
    rosbag_module = require_ros1_bag_module()
    info_topic_names = {
        topic.name: camera_id_from_topic(topic.name)
        for topic in topics.values()
        if is_camera_info_type(topic.msg_type)
    }
    camera_info: dict[str, CameraInfoRecord] = {}
    if not info_topic_names:
        return camera_info

    for bag_file in bag_files:
        with rosbag_module.Bag(str(bag_file), "r") as bag:
            for topic_name, msg, _bag_time in bag.read_messages(topics=sorted(info_topic_names)):
                camera_id = info_topic_names.get(topic_name)
                if camera_id is None or camera_id in camera_info:
                    continue
                camera_info[camera_id] = camera_info_record_from_msg(msg)
                if len(camera_info) == len(set(info_topic_names.values())):
                    return camera_info
    return camera_info


def load_camera_info_from_bag(
    source: BagSource,
    topics: dict[int, TopicRecord],
) -> dict[str, CameraInfoRecord]:
    if source.storage == "ros1":
        return load_camera_info_from_ros1_bag(source.ros1_bag_files, topics)
    return load_camera_info_from_ros2_bag(source.db3_files, topics)


def load_camera_info_from_yaml(path: Path) -> dict[str, CameraInfoRecord]:
    if not path.exists():
        return {}

    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}

    result: dict[str, CameraInfoRecord] = {}
    entries = payload.get("camera_info_by_serial")
    if isinstance(entries, dict):
        for entry in entries.values():
            camera_name = entry.get("camera_name")
            info = entry.get("camera_info")
            calibration = entry.get("calibration", {})
            if not camera_name or not isinstance(info, dict):
                continue
            result[str(camera_name)] = CameraInfoRecord(
                width=int(calibration.get("image_width", 0)),
                height=int(calibration.get("image_height", 0)),
                distortion_model=str(info.get("distortion_model", "")),
                d=[float(value) for value in info.get("d", [])],
                k=[float(value) for value in info.get("k", [])],
                r=[float(value) for value in info.get("r", [])],
                p=[float(value) for value in info.get("p", [])],
            )
    elif "camera_info" in payload:
        info = payload["camera_info"]
        result["camera"] = CameraInfoRecord(
            width=int(payload.get("image_width", 0)),
            height=int(payload.get("image_height", 0)),
            distortion_model=str(info.get("distortion_model", "")),
            d=[float(value) for value in info.get("d", [])],
            k=[float(value) for value in info.get("k", [])],
            r=[float(value) for value in info.get("r", [])],
            p=[float(value) for value in info.get("p", [])],
        )
    return result


def load_extrinsics(path: Path) -> dict[str, dict[str, list[float]]]:
    if not path.exists():
        return {}

    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}

    result: dict[str, dict[str, list[float]]] = {}
    entries = payload.get("extrinsics_by_serial", {})
    if not isinstance(entries, dict):
        return result

    for entry in entries.values():
        camera_name = entry.get("camera_name")
        if not camera_name:
            continue
        translation = [float(value) for value in entry.get("translation_xyz_m", [0.0, 0.0, 0.0])]
        rotation_xyzw = [float(value) for value in entry.get("rotation_xyzw", [0.0, 0.0, 0.0, 1.0])]
        if len(translation) != 3 or len(rotation_xyzw) != 4:
            continue
        x, y, z, w = rotation_xyzw
        result[str(camera_name)] = {
            "translation": translation,
            "rotation": [w, x, y, z],
        }
    return result


def compressed_extension(data: bytes, declared_format: str) -> tuple[str, str]:
    normalized = declared_format.lower()
    if data.startswith(b"\xff\xd8"):
        return ".jpg", "jpg"
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return ".png", "png"
    if "png" in normalized:
        return ".png", "png"
    if "jpg" in normalized or "jpeg" in normalized:
        return ".jpg", "jpg"
    return ".bin", "bin"


BAYER_ENCODINGS = {
    "bayer_rggb8",
    "bayer_rggb16",
    "bayer_bggr8",
    "bayer_bggr16",
    "bayer_gbrg8",
    "bayer_gbrg16",
    "bayer_grbg8",
    "bayer_grbg16",
}

BAYER_PATTERNS = {
    "bayer_rggb8": (("r", "g"), ("g", "b")),
    "bayer_rggb16": (("r", "g"), ("g", "b")),
    "bayer_bggr8": (("b", "g"), ("g", "r")),
    "bayer_bggr16": (("b", "g"), ("g", "r")),
    "bayer_gbrg8": (("g", "b"), ("r", "g")),
    "bayer_gbrg16": (("g", "b"), ("r", "g")),
    "bayer_grbg8": (("g", "r"), ("b", "g")),
    "bayer_grbg16": (("g", "r"), ("b", "g")),
}


def is_bayer_encoding(encoding: str) -> bool:
    return encoding.lower() in BAYER_ENCODINGS


def colorize_bayer_mosaic(image: np.ndarray, encoding: str) -> np.ndarray:
    pattern = BAYER_PATTERNS.get(encoding.lower())
    if pattern is None:
        return image
    if image.ndim != 2:
        return image

    color = np.zeros((image.shape[0], image.shape[1], 3), dtype=image.dtype)
    channel_by_color = {"b": 0, "g": 1, "r": 2}
    for row_offset, row in enumerate(pattern):
        for col_offset, color_name in enumerate(row):
            channel = channel_by_color[color_name]
            color[row_offset::2, col_offset::2, channel] = image[row_offset::2, col_offset::2]
    return color


def decode_raw_image(msg: Image, preserve_bayer: bool = False) -> np.ndarray:
    encoding = msg.encoding.lower()
    dtype = np.uint16 if encoding.endswith("16") else np.uint8
    channels = 1
    if encoding in {"rgb8", "bgr8"}:
        channels = 3
    elif encoding in {"rgba8", "bgra8"}:
        channels = 4

    array = np.frombuffer(msg.data, dtype=dtype)
    expected_row_values = int(msg.width) * channels
    row_values = int(msg.step) // np.dtype(dtype).itemsize if msg.step else expected_row_values
    if row_values < expected_row_values:
        raise RuntimeError(
            f"Image step is too small for {msg.encoding}: step={msg.step}, width={msg.width}"
        )
    required_values = int(msg.height) * row_values
    if array.size < required_values:
        raise RuntimeError(
            f"Image data is too small for {msg.encoding}: got {array.size} values, "
            f"need {required_values}"
        )
    array = array[:required_values].reshape((msg.height, row_values))[:, :expected_row_values]
    if channels > 1:
        array = array.reshape((msg.height, msg.width, channels))
    else:
        array = array.reshape((msg.height, msg.width))

    if preserve_bayer and is_bayer_encoding(encoding):
        return array
    if encoding == "rgb8":
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(array, cv2.COLOR_RGBA2BGRA)
    if encoding in {"bayer_rggb8", "bayer_rggb16"}:
        return cv2.cvtColor(array, cv2.COLOR_BAYER_RG2BGR)
    if encoding in {"bayer_bggr8", "bayer_bggr16"}:
        return cv2.cvtColor(array, cv2.COLOR_BAYER_BG2BGR)
    if encoding in {"bayer_gbrg8", "bayer_gbrg16"}:
        return cv2.cvtColor(array, cv2.COLOR_BAYER_GB2BGR)
    if encoding in {"bayer_grbg8", "bayer_grbg16"}:
        return cv2.cvtColor(array, cv2.COLOR_BAYER_GR2BGR)
    return array


def undistort_image(
    image: np.ndarray,
    camera_info: CameraInfoRecord | None,
    camera_id: str,
    interpolation: int = cv2.INTER_LINEAR,
) -> np.ndarray:
    if camera_info is None:
        raise RuntimeError(f"Cannot undistort '{camera_id}' because no CameraInfo was found.")
    if len(camera_info.k) != 9:
        raise RuntimeError(f"Cannot undistort '{camera_id}' because CameraInfo.K is not 3x3.")
    if not camera_info.d:
        raise RuntimeError(f"Cannot undistort '{camera_id}' because CameraInfo.D is empty.")

    intrinsic = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.array(camera_info.d, dtype=np.float64)
    if interpolation == cv2.INTER_LINEAR:
        return cv2.undistort(image, intrinsic, distortion, None, intrinsic)

    map_x, map_y = cv2.initUndistortRectifyMap(
        intrinsic,
        distortion,
        None,
        intrinsic,
        (image.shape[1], image.shape[0]),
        cv2.CV_32FC1,
    )
    return cv2.remap(image, map_x, map_y, interpolation)


def build_stream_maps(
    selected_topics: list[TopicRecord],
    channel_overrides: dict[str, str],
) -> tuple[
    dict[int, str],
    dict[int, str],
    dict[int, str],
    dict[str, str],
    dict[str, str],
    dict[str, list[ImageRecord]],
]:
    camera_by_topic = {topic.topic_id: camera_id_from_topic(topic.name) for topic in selected_topics}
    stream_kind_by_topic = {
        topic.topic_id: stream_kind_from_topic(topic.name) for topic in selected_topics
    }
    topics_by_camera: dict[str, list[int]] = {}
    for topic_id, camera_id in camera_by_topic.items():
        topics_by_camera.setdefault(camera_id, []).append(topic_id)

    stream_by_topic = {
        topic_id: stream_id_for(
            camera_id,
            stream_kind_by_topic[topic_id],
            len(topics_by_camera[camera_id]) > 1,
        )
        for topic_id, camera_id in camera_by_topic.items()
    }
    camera_by_stream = {
        stream_id: camera_by_topic[topic_id]
        for topic_id, stream_id in stream_by_topic.items()
    }
    channel_by_stream = {
        stream_id: default_stream_channel(
            camera_by_topic[topic_id],
            stream_id,
            stream_kind_by_topic[topic_id],
            len(topics_by_camera[camera_by_topic[topic_id]]) > 1,
            channel_overrides,
        )
        for topic_id, stream_id in stream_by_topic.items()
    }
    frames_by_stream = {stream_id: [] for stream_id in channel_by_stream}
    return (
        camera_by_topic,
        stream_kind_by_topic,
        stream_by_topic,
        camera_by_stream,
        channel_by_stream,
        frames_by_stream,
    )


def read_images_ros2(
    db3_files: list[Path],
    selected_topics: list[TopicRecord],
    channel_overrides: dict[str, str],
) -> tuple[dict[str, list[ImageRecord]], dict[str, str], dict[str, str]]:
    topic_by_id = {topic.topic_id: topic for topic in selected_topics}
    (
        camera_by_topic,
        stream_kind_by_topic,
        stream_by_topic,
        camera_by_stream,
        channel_by_stream,
        frames_by_stream,
    ) = build_stream_maps(selected_topics, channel_overrides)

    for db3_file in db3_files:
        with sqlite3.connect(db3_file) as connection:
            for topic_id, bag_timestamp_ns, data in connection.execute(
                "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
            ):
                topic = topic_by_id.get(int(topic_id))
                if topic is None:
                    continue

                camera_id = camera_by_topic[int(topic_id)]
                stream_id = stream_by_topic[int(topic_id)]
                stream_kind = stream_kind_by_topic[int(topic_id)]
                channel = channel_by_stream[stream_id]
                if is_compressed_image_type(topic.msg_type):
                    msg = require_ros2_deserializer()(data, CompressedImage)
                    frames_by_stream[stream_id].append(
                        ImageRecord(
                            stream_id=stream_id,
                            camera_id=camera_id,
                            stream_kind=stream_kind,
                            channel=channel,
                            topic=topic.name,
                            msg_type=topic.msg_type,
                            timestamp_ns=stamp_to_ns(msg.header.stamp, int(bag_timestamp_ns)),
                            bag_timestamp_ns=int(bag_timestamp_ns),
                            data=bytes(msg.data),
                            format=str(msg.format or "jpeg"),
                        )
                    )
                elif is_raw_image_type(topic.msg_type):
                    msg = require_ros2_deserializer()(data, Image)
                    frames_by_stream[stream_id].append(
                        ImageRecord(
                            stream_id=stream_id,
                            camera_id=camera_id,
                            stream_kind=stream_kind,
                            channel=channel,
                            topic=topic.name,
                            msg_type=topic.msg_type,
                            timestamp_ns=stamp_to_ns(msg.header.stamp, int(bag_timestamp_ns)),
                            bag_timestamp_ns=int(bag_timestamp_ns),
                            data=bytes(data),
                            format=str(msg.encoding),
                            width=int(msg.width),
                            height=int(msg.height),
                        )
                    )

    for frames in frames_by_stream.values():
        frames.sort(key=lambda frame: frame.timestamp_ns)
    return frames_by_stream, channel_by_stream, camera_by_stream


def read_images_ros1(
    bag_files: list[Path],
    selected_topics: list[TopicRecord],
    channel_overrides: dict[str, str],
) -> tuple[dict[str, list[ImageRecord]], dict[str, str], dict[str, str]]:
    rosbag_module = require_ros1_bag_module()
    topic_by_name = {topic.name: topic for topic in selected_topics}
    topic_id_by_name = {topic.name: topic.topic_id for topic in selected_topics}
    (
        camera_by_topic,
        stream_kind_by_topic,
        stream_by_topic,
        camera_by_stream,
        channel_by_stream,
        frames_by_stream,
    ) = build_stream_maps(selected_topics, channel_overrides)

    for bag_file in bag_files:
        with rosbag_module.Bag(str(bag_file), "r") as bag:
            for topic_name, msg, bag_time in bag.read_messages(topics=sorted(topic_by_name)):
                topic = topic_by_name.get(topic_name)
                if topic is None:
                    continue

                topic_id = topic_id_by_name[topic_name]
                bag_timestamp_ns = int(bag_time.to_nsec())
                camera_id = camera_by_topic[topic_id]
                stream_id = stream_by_topic[topic_id]
                stream_kind = stream_kind_by_topic[topic_id]
                channel = channel_by_stream[stream_id]
                if is_compressed_image_type(topic.msg_type):
                    frames_by_stream[stream_id].append(
                        ImageRecord(
                            stream_id=stream_id,
                            camera_id=camera_id,
                            stream_kind=stream_kind,
                            channel=channel,
                            topic=topic.name,
                            msg_type=topic.msg_type,
                            timestamp_ns=stamp_to_ns(msg.header.stamp, bag_timestamp_ns),
                            bag_timestamp_ns=bag_timestamp_ns,
                            data=bytes(msg.data),
                            format=str(msg.format or "jpeg"),
                        )
                    )
                elif is_raw_image_type(topic.msg_type):
                    frames_by_stream[stream_id].append(
                        ImageRecord(
                            stream_id=stream_id,
                            camera_id=camera_id,
                            stream_kind=stream_kind,
                            channel=channel,
                            topic=topic.name,
                            msg_type=topic.msg_type,
                            timestamp_ns=stamp_to_ns(msg.header.stamp, bag_timestamp_ns),
                            bag_timestamp_ns=bag_timestamp_ns,
                            data=msg,
                            format=str(msg.encoding),
                            width=int(msg.width),
                            height=int(msg.height),
                        )
                    )

    for frames in frames_by_stream.values():
        frames.sort(key=lambda frame: frame.timestamp_ns)
    return frames_by_stream, channel_by_stream, camera_by_stream


def read_images(
    source: BagSource,
    selected_topics: list[TopicRecord],
    channel_overrides: dict[str, str],
) -> tuple[dict[str, list[ImageRecord]], dict[str, str], dict[str, str]]:
    if source.storage == "ros1":
        return read_images_ros1(source.ros1_bag_files, selected_topics, channel_overrides)
    return read_images_ros2(source.db3_files, selected_topics, channel_overrides)


def group_samples(
    frames_by_camera: dict[str, list[ImageRecord]],
    tolerance_ms: float,
    limit: int,
) -> list[dict[str, ImageRecord]]:
    cameras = sorted(frames_by_camera)
    if not cameras:
        return []
    reference_camera = max(cameras, key=lambda camera: (len(frames_by_camera[camera]), -cameras.index(camera)))
    reference_frames = frames_by_camera[reference_camera]
    tolerance_ns = int(tolerance_ms * 1_000_000)
    cursors = {camera: 0 for camera in cameras if camera != reference_camera}
    groups: list[dict[str, ImageRecord]] = []

    for reference_frame in reference_frames:
        group = {reference_camera: reference_frame}
        ref_ts = reference_frame.timestamp_ns

        for camera in cameras:
            if camera == reference_camera:
                continue
            frames = frames_by_camera[camera]
            if not frames:
                continue

            cursor = cursors[camera]
            while cursor < len(frames) and frames[cursor].timestamp_ns < ref_ts - tolerance_ns:
                cursor += 1

            candidates = []
            if cursor < len(frames):
                candidates.append(cursor)
            if cursor > 0:
                candidates.append(cursor - 1)

            best_index = None
            best_delta = None
            for candidate in candidates:
                delta = abs(frames[candidate].timestamp_ns - ref_ts)
                if best_delta is None or delta < best_delta:
                    best_delta = delta
                    best_index = candidate

            if best_index is not None and best_delta is not None and best_delta <= tolerance_ns:
                group[camera] = frames[best_index]
                cursors[camera] = best_index + 1
            else:
                cursors[camera] = cursor

        groups.append(group)
        if limit > 0 and len(groups) >= limit:
            break

    return groups


def prepare_output(output: Path, overwrite: bool) -> None:
    if output.exists():
        if not overwrite:
            raise FileExistsError(
                f"Output directory already exists: {output}. Use --overwrite to replace it."
            )
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)


def write_blank_map(output: Path) -> str:
    maps_dir = output / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)
    filename = "maps/flir_blank_map.png"
    path = output / filename
    cv2.imwrite(str(path), np.zeros((1, 1), dtype=np.uint8))
    return filename


def write_image_file(
    output: Path,
    record: ImageRecord,
    filename_base: str,
    camera_info: CameraInfoRecord | None,
    undistorted_export: bool,
    preserve_bayer_raw: bool,
) -> tuple[str, str, int, int]:
    channel_dir = output / "samples" / record.channel
    channel_dir.mkdir(parents=True, exist_ok=True)
    should_undistort = undistorted_export and not is_undistorted_topic(record.topic)
    should_preserve_bayer = preserve_bayer_raw and record.stream_kind == "raw"
    if should_preserve_bayer:
        should_undistort = False

    if is_compressed_image_type(record.msg_type):
        extension, fileformat = compressed_extension(record.data, record.format)
        relative_path = Path("samples") / record.channel / f"{filename_base}{extension}"
        image_path = output / relative_path
        decoded = cv2.imdecode(np.frombuffer(record.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if decoded is None:
            if should_undistort:
                raise RuntimeError(f"Failed to decode compressed image for undistortion: {record.topic}")
            image_path.write_bytes(record.data)
            height = record.height or 0
            width = record.width or 0
        else:
            if should_undistort:
                decoded = undistort_image(decoded, camera_info, record.camera_id)
                write_options = []
                if fileformat == "jpg":
                    write_options = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
                if not cv2.imwrite(str(image_path), decoded, write_options):
                    raise RuntimeError(f"Failed to write undistorted image: {image_path}")
            else:
                image_path.write_bytes(record.data)
            height, width = decoded.shape[:2]
        return relative_path.as_posix(), fileformat, width, height

    if is_ros_image_message(record.data):
        msg = record.data
    else:
        msg = require_ros2_deserializer()(record.data, Image)
    image = decode_raw_image(msg, preserve_bayer=should_preserve_bayer)
    if should_preserve_bayer:
        image = colorize_bayer_mosaic(image, msg.encoding)
    if should_undistort:
        interpolation = cv2.INTER_NEAREST if should_preserve_bayer else cv2.INTER_LINEAR
        image = undistort_image(image, camera_info, record.camera_id, interpolation)
    relative_path = Path("samples") / record.channel / f"{filename_base}.png"
    image_path = output / relative_path
    if not cv2.imwrite(str(image_path), image):
        raise RuntimeError(f"Failed to write image: {image_path}")
    height, width = image.shape[:2]
    return relative_path.as_posix(), "png", width, height


def table_path(output: Path, version: str, table_name: str) -> Path:
    return output / version / f"{table_name}.json"


def write_json_tables(output: Path, version: str, tables: dict[str, list[dict[str, Any]]]) -> None:
    table_root = output / version
    table_root.mkdir(parents=True, exist_ok=True)
    for name in [
        "category",
        "attribute",
        "visibility",
        "instance",
        "sensor",
        "calibrated_sensor",
        "ego_pose",
        "log",
        "scene",
        "sample",
        "sample_data",
        "sample_annotation",
        "map",
    ]:
        with table_path(output, version, name).open("w", encoding="utf-8") as stream:
            json.dump(tables.get(name, []), stream, indent=2)
            stream.write("\n")


def build_tables(
    output: Path,
    version: str,
    bag_name: str,
    scene_name: str,
    description: str,
    groups: list[dict[str, ImageRecord]],
    channel_by_stream: dict[str, str],
    camera_by_stream: dict[str, str],
    camera_info: dict[str, CameraInfoRecord],
    extrinsics: dict[str, dict[str, list[float]]],
    undistorted_export: bool,
    preserve_bayer_raw: bool,
) -> tuple[dict[str, list[dict[str, Any]]], dict[str, int]]:
    log_token = token_for("log", bag_name)
    scene_token = token_for("scene", bag_name, scene_name)
    map_token = token_for("map", bag_name)
    map_filename = write_blank_map(output)

    sensors = []
    calibrated_sensors = []
    for stream_id, channel in sorted(channel_by_stream.items(), key=lambda item: item[1]):
        camera_id = camera_by_stream.get(stream_id, stream_id)
        sensor_token = token_for("sensor", channel)
        calibrated_sensor_token = token_for("calibrated_sensor", channel)
        info = camera_info.get(camera_id)
        extrinsic = extrinsics.get(camera_id, {})
        sensors.append(
            {
                "token": sensor_token,
                "channel": channel,
                "modality": "camera",
            }
        )
        calibrated_sensors.append(
            {
                "token": calibrated_sensor_token,
                "sensor_token": sensor_token,
                "translation": extrinsic.get("translation", [0.0, 0.0, 0.0]),
                "rotation": extrinsic.get("rotation", [1.0, 0.0, 0.0, 0.0]),
                "camera_intrinsic": info.intrinsic_3x3 if info else [],
            }
        )

    sample_records = []
    sample_data_records = []
    ego_pose_records = []
    frames_written_by_channel = {channel: 0 for channel in channel_by_stream.values()}

    sample_tokens: list[str] = []
    per_channel_sample_data_tokens: dict[str, list[str]] = {
        channel: [] for channel in channel_by_stream.values()
    }

    for sample_index, group in enumerate(groups):
        timestamp_ns = min(frame.timestamp_ns for frame in group.values())
        timestamp_us = ns_to_us(timestamp_ns)
        sample_token = token_for("sample", bag_name, sample_index, timestamp_us)
        sample_tokens.append(sample_token)
        sample_records.append(
            {
                "token": sample_token,
                "timestamp": timestamp_us,
                "prev": "",
                "next": "",
                "scene_token": scene_token,
            }
        )

        ego_pose_token = token_for("ego_pose", bag_name, timestamp_us)
        ego_pose_records.append(
            {
                "token": ego_pose_token,
                "timestamp": timestamp_us,
                "rotation": [1.0, 0.0, 0.0, 0.0],
                "translation": [0.0, 0.0, 0.0],
            }
        )

        for _stream_id, frame in sorted(group.items(), key=lambda item: item[1].channel):
            channel = frame.channel
            sample_data_token = token_for(
                "sample_data",
                bag_name,
                channel,
                sample_index,
                ns_to_us(frame.timestamp_ns),
            )
            filename_base = f"{sample_token}__{channel}__{ns_to_us(frame.timestamp_ns)}"
            info = camera_info.get(frame.camera_id)
            filename, fileformat, width, height = write_image_file(
                output,
                frame,
                filename_base,
                info,
                undistorted_export,
                preserve_bayer_raw,
            )
            sample_data_records.append(
                {
                    "token": sample_data_token,
                    "sample_token": sample_token,
                    "ego_pose_token": ego_pose_token,
                    "calibrated_sensor_token": token_for("calibrated_sensor", channel),
                    "timestamp": ns_to_us(frame.timestamp_ns),
                    "fileformat": fileformat,
                    "is_key_frame": True,
                    "height": int(info.height if info and info.height else height),
                    "width": int(info.width if info and info.width else width),
                    "filename": filename,
                    "prev": "",
                    "next": "",
                }
            )
            per_channel_sample_data_tokens[channel].append(sample_data_token)
            frames_written_by_channel[channel] += 1

    for index, sample in enumerate(sample_records):
        sample["prev"] = sample_tokens[index - 1] if index > 0 else ""
        sample["next"] = sample_tokens[index + 1] if index + 1 < len(sample_tokens) else ""

    sample_data_by_token = {record["token"]: record for record in sample_data_records}
    for tokens in per_channel_sample_data_tokens.values():
        for index, token in enumerate(tokens):
            record = sample_data_by_token[token]
            record["prev"] = tokens[index - 1] if index > 0 else ""
            record["next"] = tokens[index + 1] if index + 1 < len(tokens) else ""

    first_timestamp_us = sample_records[0]["timestamp"] if sample_records else 0
    date_captured = (
        datetime.fromtimestamp(first_timestamp_us / 1_000_000, tz=timezone.utc)
        .date()
        .isoformat()
        if first_timestamp_us
        else ""
    )

    tables: dict[str, list[dict[str, Any]]] = {
        "category": [],
        "attribute": [],
        "visibility": [],
        "instance": [],
        "sensor": sensors,
        "calibrated_sensor": calibrated_sensors,
        "ego_pose": ego_pose_records,
        "log": [
            {
                "token": log_token,
                "logfile": bag_name,
                "vehicle": "flir_multicam_rig",
                "date_captured": date_captured,
                "location": "flir_capture",
            }
        ],
        "scene": [
            {
                "token": scene_token,
                "name": scene_name,
                "description": description,
                "log_token": log_token,
                "nbr_samples": len(sample_records),
                "first_sample_token": sample_tokens[0] if sample_tokens else "",
                "last_sample_token": sample_tokens[-1] if sample_tokens else "",
            }
        ],
        "sample": sample_records,
        "sample_data": sample_data_records,
        "sample_annotation": [],
        "map": [
            {
                "token": map_token,
                "category": "semantic_prior",
                "filename": map_filename,
                "log_tokens": [log_token],
            }
        ],
    }
    return tables, frames_written_by_channel


def write_summary(summary: ConversionSummary) -> None:
    payload = {
        "bag": str(summary.bag),
        "output": str(summary.output),
        "version": summary.version,
        "scene_name": summary.scene_name,
        "selected_topics": summary.selected_topics,
        "channel_by_stream": summary.channel_by_stream,
        "camera_by_stream": summary.camera_by_stream,
        "frames_read_by_stream": summary.frames_read_by_stream,
        "frames_written_by_channel": summary.frames_written_by_channel,
        "samples_written": summary.samples_written,
        "sync_tolerance_ms": summary.sync_tolerance_ms,
        "undistorted_export": summary.undistorted_export,
        "preserve_bayer_raw": summary.preserve_bayer_raw,
    }
    with (summary.output / "conversion_report.json").open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)
        stream.write("\n")


def print_summary(summary: ConversionSummary) -> None:
    print(f"bag: {summary.bag}")
    print(f"output: {summary.output}")
    print(f"version: {summary.version}")
    print(f"scene: {summary.scene_name}")
    print("topics:")
    for topic in summary.selected_topics:
        print(f"  - {topic}")
    print("streams:")
    for stream_id, channel in sorted(summary.channel_by_stream.items()):
        camera_id = summary.camera_by_stream.get(stream_id, stream_id)
        read_count = summary.frames_read_by_stream.get(stream_id, 0)
        written_count = summary.frames_written_by_channel.get(channel, 0)
        print(f"  - {stream_id} ({camera_id}) -> {channel}: read {read_count}, wrote {written_count}")
    print(f"samples: {summary.samples_written}")
    print(f"undistorted export: {summary.undistorted_export}")
    print(f"preserve Bayer raw: {summary.preserve_bayer_raw}")


def main() -> None:
    args = parse_args()
    if args.raw_bayer and args.raw_and_rgb:
        raise ValueError("Use only one of --raw-bayer or --raw-and-rgb.")

    root = workspace_root()
    bag_source = resolve_bag(args.bag, root)
    output = (
        Path(args.output).expanduser()
        if args.output
        else root / "nuscenes_export" / bag_source.output_name
    )
    if not output.is_absolute():
        output = (root / output).resolve()

    topics = load_topics(bag_source)
    selected_topics = choose_image_topics(topics, args.image_topic, args.raw_bayer, args.raw_and_rgb)
    channel_overrides = parse_channel_overrides(args.camera_channel)
    frames_by_stream, channel_by_stream, camera_by_stream = read_images(
        bag_source,
        selected_topics,
        channel_overrides,
    )
    groups = group_samples(frames_by_stream, args.sync_tolerance_ms, args.limit)
    if not groups:
        raise RuntimeError("No synchronized image samples could be formed from selected topics.")

    camera_info = load_camera_info_from_yaml((root / args.camera_info_yaml).resolve())
    camera_info.update(load_camera_info_from_bag(bag_source, topics))
    extrinsics = load_extrinsics((root / args.extrinsics_yaml).resolve())
    scene_name = args.scene_name or bag_source.output_name
    undistorted_export = not args.no_undistort
    preserve_bayer_raw = args.raw_bayer or args.raw_and_rgb
    frames_read_by_stream = {
        stream_id: len(frames)
        for stream_id, frames in sorted(frames_by_stream.items())
    }

    if args.dry_run:
        summary = ConversionSummary(
            bag=bag_source.path,
            output=output,
            version=args.version,
            scene_name=scene_name,
            selected_topics=[topic.name for topic in selected_topics],
            channel_by_stream=channel_by_stream,
            camera_by_stream=camera_by_stream,
            frames_read_by_stream=frames_read_by_stream,
            frames_written_by_channel={channel: 0 for channel in channel_by_stream.values()},
            samples_written=len(groups),
            sync_tolerance_ms=args.sync_tolerance_ms,
            undistorted_export=undistorted_export,
            preserve_bayer_raw=preserve_bayer_raw,
        )
        print_summary(summary)
        return

    prepare_output(output, args.overwrite)
    tables, frames_written_by_channel = build_tables(
        output=output,
        version=args.version,
        bag_name=bag_source.output_name,
        scene_name=scene_name,
        description=args.description,
        groups=groups,
        channel_by_stream=channel_by_stream,
        camera_by_stream=camera_by_stream,
        camera_info=camera_info,
        extrinsics=extrinsics,
        undistorted_export=undistorted_export,
        preserve_bayer_raw=preserve_bayer_raw,
    )
    write_json_tables(output, args.version, tables)

    summary = ConversionSummary(
        bag=bag_source.path,
        output=output,
        version=args.version,
        scene_name=scene_name,
        selected_topics=[topic.name for topic in selected_topics],
        channel_by_stream=channel_by_stream,
        camera_by_stream=camera_by_stream,
        frames_read_by_stream=frames_read_by_stream,
        frames_written_by_channel=frames_written_by_channel,
        samples_written=len(groups),
        sync_tolerance_ms=args.sync_tolerance_ms,
        undistorted_export=undistorted_export,
        preserve_bayer_raw=preserve_bayer_raw,
    )
    write_summary(summary)
    print_summary(summary)


if __name__ == "__main__":
    main()
