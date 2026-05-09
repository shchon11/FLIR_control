from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


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

    return cameras


def _select_camera(cameras: list, camera_name: str, camera_serial: str) -> dict:
    if camera_serial:
        for camera in cameras:
            if camera["serial"] == camera_serial:
                return camera
        raise RuntimeError(f"camera_serial '{camera_serial}' was not found in the camera inventory")

    if camera_name:
        for camera in cameras:
            if camera["name"] == camera_name:
                return camera
        raise RuntimeError(f"camera_name '{camera_name}' was not found in the camera inventory")

    if not cameras:
        raise RuntimeError("Camera inventory is empty")
    return cameras[0]


def _load_ros_parameters(path: str, node_name: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return dict(data.get(node_name, {}).get("ros__parameters", {}))


def _build_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    cameras_file = LaunchConfiguration("cameras_file").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)
    camera_serial = LaunchConfiguration("camera_serial").perform(context)
    output_yaml_path = LaunchConfiguration("output_yaml_path").perform(context)
    sample_image_dir = LaunchConfiguration("sample_image_dir").perform(context)

    camera = _select_camera(_parse_cameras_file(cameras_file), camera_name, camera_serial)
    shared_parameters = _load_ros_parameters(params_file, "flir_camera_calibration")
    return [
        Node(
            package="flir_camera_calibration",
            executable="flir_camera_calibration_node",
            name="flir_camera_calibration",
            namespace=camera["namespace"],
            output="screen",
            parameters=[
                shared_parameters,
                {
                    "input_topic": "image_rgb/compressed",
                    "annotated_output_topic": "calibration/image_annotated/compressed",
                    "output_yaml_path": output_yaml_path,
                    "sample_image_dir": sample_image_dir,
                    "camera_serial": camera["serial"],
                    "camera_name": camera["name"],
                    "frame_id": camera["frame_id"],
                    "window_name": f"FLIR Calibration {camera['name']}",
                },
            ],
        )
    ]


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("flir_camera_calibration"), "config", "calibration.yaml"]
    )
    cameras_file = PathJoinSubstitution(
        [FindPackageShare("flir_spinnaker_camera"), "config", "multicam_cameras.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Calibration parameter file.",
            ),
            DeclareLaunchArgument(
                "cameras_file",
                default_value=cameras_file,
                description="Camera inventory YAML.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="camera0",
                description="Logical camera name to calibrate. Ignored when camera_serial is set.",
            ),
            DeclareLaunchArgument(
                "camera_serial",
                default_value="",
                description="Camera serial to calibrate.",
            ),
            DeclareLaunchArgument(
                "output_yaml_path",
                default_value="calibration/flir_camera_info.yaml",
                description="Serial-indexed intrinsic calibration YAML.",
            ),
            DeclareLaunchArgument(
                "sample_image_dir",
                default_value="calibration/captures",
                description="Base directory for captured calibration frames. The serial is appended.",
            ),
            OpaqueFunction(function=_build_node),
        ]
    )
