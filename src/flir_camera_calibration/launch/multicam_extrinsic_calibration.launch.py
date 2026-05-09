from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Expected a boolean launch value, got: {value}")


def _optional_override(context, name, parser=None):
    value = LaunchConfiguration(name).perform(context)
    if value == "":
        return None
    return parser(value) if parser is not None else value


def _strip_quotes(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


def _apply_key_value(camera: dict, text: str) -> None:
    if ":" not in text:
        return
    key, value = text.split(":", 1)
    key = key.strip()
    value = _strip_quotes(value.strip())
    if key:
        camera[key] = value


def _parse_cameras_file(path: str) -> list:
    cameras = []
    current = None
    in_cameras = False

    with open(path, "r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.split("#", 1)[0].rstrip()
            stripped = line.strip()
            if not stripped:
                continue
            if stripped == "cameras:":
                in_cameras = True
                continue
            if not in_cameras:
                continue
            if stripped.startswith("- "):
                if current is not None:
                    cameras.append(current)
                current = {}
                _apply_key_value(current, stripped[2:].strip())
                continue
            if current is not None:
                _apply_key_value(current, stripped)

    if current is not None:
        cameras.append(current)

    for index, camera in enumerate(cameras):
        camera.setdefault("name", f"camera{index}")
        camera.setdefault("namespace", camera["name"])
        camera.setdefault("frame_id", f"{camera['name']}_optical_frame")
        if not camera.get("serial"):
            raise RuntimeError(f"Camera '{camera['name']}' is missing a serial in {path}")

    if len(cameras) < 2:
        raise RuntimeError("At least two cameras are required for extrinsic calibration")
    return cameras


def _load_ros_parameters(path: str, node_name: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return dict(data.get(node_name, {}).get("ros__parameters", {}))


def _build_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    cameras_file = LaunchConfiguration("cameras_file").perform(context)
    output_yaml_path = LaunchConfiguration("output_yaml_path").perform(context)
    reference_camera = LaunchConfiguration("reference_camera").perform(context)
    reference_frame = LaunchConfiguration("reference_frame").perform(context)
    cameras = _parse_cameras_file(cameras_file)
    shared_parameters = _load_ros_parameters(params_file, "flir_camera_extrinsic_calibration")
    parameter_overrides = {}

    for name in (
        "image_topic",
        "camera_info_topic",
        "window_name",
        "input_qos_reliability",
        "camera_info_qos_reliability",
    ):
        value = _optional_override(context, name)
        if value is not None:
            parameter_overrides[name] = value

    for name in ("display_window", "preview_fast_check"):
        value = _optional_override(context, name, _parse_bool)
        if value is not None:
            parameter_overrides[name] = value

    for name in (
        "board_cols",
        "board_rows",
        "min_observations",
        "max_frame_age_ms",
        "preview_max_width",
        "input_qos_depth",
        "camera_info_qos_depth",
    ):
        value = _optional_override(context, name, int)
        if value is not None:
            parameter_overrides[name] = value

    value = _optional_override(context, "square_size_m", float)
    if value is not None:
        parameter_overrides["square_size_m"] = value

    return [
        Node(
            package="flir_camera_calibration",
            executable="flir_camera_extrinsic_calibration_node",
            name="flir_camera_extrinsic_calibration",
            output="screen",
            parameters=[
                shared_parameters,
                {
                    "camera_namespaces": [camera["namespace"] for camera in cameras],
                    "camera_names": [camera["name"] for camera in cameras],
                    "camera_serials": [camera["serial"] for camera in cameras],
                    "camera_frame_ids": [camera["frame_id"] for camera in cameras],
                    "reference_camera": reference_camera,
                    "reference_frame": reference_frame,
                    "output_yaml_path": output_yaml_path,
                },
                parameter_overrides,
            ],
        )
    ]


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("flir_camera_calibration"), "config", "extrinsic_calibration.yaml"]
    )
    cameras_file = PathJoinSubstitution(
        [FindPackageShare("flir_spinnaker_camera"), "config", "multicam_cameras.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Extrinsic calibration parameter file.",
            ),
            DeclareLaunchArgument(
                "cameras_file",
                default_value=cameras_file,
                description="Camera inventory YAML.",
            ),
            DeclareLaunchArgument(
                "output_yaml_path",
                default_value="calibration/flir_camera_extrinsics.yaml",
                description="Extrinsic calibration output YAML.",
            ),
            DeclareLaunchArgument(
                "reference_camera",
                default_value="camera0",
                description="Reference camera name, namespace, or serial. The rig frame coincides with it.",
            ),
            DeclareLaunchArgument(
                "reference_frame",
                default_value="flir_rig_frame",
                description="Parent frame written to the extrinsics YAML.",
            ),
            DeclareLaunchArgument("image_topic", default_value="", description="Override relative image topic."),
            DeclareLaunchArgument("camera_info_topic", default_value="", description="Override relative camera info topic."),
            DeclareLaunchArgument("board_cols", default_value="", description="Override chessboard inner-corner columns."),
            DeclareLaunchArgument("board_rows", default_value="", description="Override chessboard inner-corner rows."),
            DeclareLaunchArgument("square_size_m", default_value="", description="Override chessboard square size in meters."),
            DeclareLaunchArgument("min_observations", default_value="", description="Override minimum observations."),
            DeclareLaunchArgument("max_frame_age_ms", default_value="", description="Override freshness limit for capture."),
            DeclareLaunchArgument("display_window", default_value="", description="Override OpenCV preview visibility."),
            DeclareLaunchArgument("window_name", default_value="", description="Override OpenCV preview window title."),
            DeclareLaunchArgument("preview_max_width", default_value="", description="Override preview detection max width."),
            DeclareLaunchArgument("preview_fast_check", default_value="", description="Override preview FAST_CHECK usage."),
            DeclareLaunchArgument(
                "input_qos_reliability",
                default_value="",
                description="Override input QoS reliability: best_effort or reliable.",
            ),
            DeclareLaunchArgument("input_qos_depth", default_value="", description="Override input QoS queue depth."),
            DeclareLaunchArgument(
                "camera_info_qos_reliability",
                default_value="",
                description="Override camera_info QoS reliability: best_effort or reliable.",
            ),
            DeclareLaunchArgument("camera_info_qos_depth", default_value="", description="Override camera_info QoS depth."),
            OpaqueFunction(function=_build_node),
        ]
    )
