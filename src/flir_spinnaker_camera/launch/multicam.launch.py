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


def _load_ros_parameters(path: str, node_name: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return dict(data.get(node_name, {}).get("ros__parameters", {}))


def _build_camera_nodes(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    cameras_file = LaunchConfiguration("cameras_file").perform(context)
    camera_info_yaml_path = LaunchConfiguration("camera_info_yaml_path").perform(context)
    cameras = _parse_cameras_file(cameras_file)
    shared_parameters = _load_ros_parameters(params_file, "flir_camera")

    nodes = []
    for camera in cameras:
        nodes.append(
            Node(
                package="flir_spinnaker_camera",
                executable="flir_spinnaker_camera_node",
                name="flir_camera",
                namespace=camera["namespace"],
                output="screen",
                parameters=[
                    shared_parameters,
                    {
                        "camera_serial": camera["serial"],
                        "frame_id": camera["frame_id"],
                        "camera_info.yaml_path": camera_info_yaml_path,
                    },
                ],
            )
        )
    return nodes


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("flir_spinnaker_camera"), "config", "flir_camera.yaml"]
    )
    cameras_file = PathJoinSubstitution(
        [FindPackageShare("flir_spinnaker_camera"), "config", "multicam_cameras.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Shared FLIR camera parameter file used by every camera.",
            ),
            DeclareLaunchArgument(
                "cameras_file",
                default_value=cameras_file,
                description="Camera inventory YAML with name, serial, namespace, and frame_id.",
            ),
            DeclareLaunchArgument(
                "camera_info_yaml_path",
                default_value="calibration/flir_camera_info.yaml",
                description="Serial-indexed intrinsic calibration YAML.",
            ),
            OpaqueFunction(function=_build_camera_nodes),
        ]
    )
