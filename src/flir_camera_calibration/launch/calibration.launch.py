from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


def _build_node(context):
    parameter_overrides = {}

    for name in (
        "input_topic",
        "annotated_output_topic",
        "output_yaml_path",
        "camera_serial",
        "camera_name",
        "frame_id",
        "sample_image_dir",
        "window_name",
        "input_qos_reliability",
        "output_qos_reliability",
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
        "min_calibration_frames",
        "annotated_jpeg_quality",
        "input_qos_depth",
        "output_qos_depth",
    ):
        value = _optional_override(context, name, int)
        if value is not None:
            parameter_overrides[name] = value

    for name in ("square_size_m", "preview_scale"):
        value = _optional_override(context, name, float)
        if value is not None:
            parameter_overrides[name] = value

    for name in ("preview_max_width",):
        value = _optional_override(context, name, int)
        if value is not None:
            parameter_overrides[name] = value

    return [
        Node(
            package="flir_camera_calibration",
            executable="flir_camera_calibration_node",
            name="flir_camera_calibration",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                parameter_overrides,
            ],
        )
    ]


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("flir_camera_calibration"), "config", "calibration.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the calibration node.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Path to the parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="",
                description="Override the compressed input topic.",
            ),
            DeclareLaunchArgument(
                "annotated_output_topic",
                default_value="",
                description="Override the annotated compressed output topic.",
            ),
            DeclareLaunchArgument(
                "output_yaml_path",
                default_value="",
                description="Override the saved calibration YAML path.",
            ),
            DeclareLaunchArgument(
                "camera_serial",
                default_value="",
                description="Optional camera serial used as the key in serial-indexed calibration YAML.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="",
                description="Optional logical camera name such as camera0.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Optional camera optical frame id stored with the calibration entry.",
            ),
            DeclareLaunchArgument(
                "sample_image_dir",
                default_value="",
                description="Override the optional directory for captured calibration frames.",
            ),
            DeclareLaunchArgument(
                "board_cols",
                default_value="",
                description="Override the chessboard inner-corner column count.",
            ),
            DeclareLaunchArgument(
                "board_rows",
                default_value="",
                description="Override the chessboard inner-corner row count.",
            ),
            DeclareLaunchArgument(
                "square_size_m",
                default_value="",
                description="Override the chessboard square size in meters.",
            ),
            DeclareLaunchArgument(
                "min_calibration_frames",
                default_value="",
                description="Override the minimum number of captured frames before calibration.",
            ),
            DeclareLaunchArgument(
                "display_window",
                default_value="",
                description="Override whether to show the OpenCV preview window.",
            ),
            DeclareLaunchArgument(
                "window_name",
                default_value="",
                description="Override the OpenCV preview window title.",
            ),
            DeclareLaunchArgument(
                "preview_scale",
                default_value="",
                description="Override the preview scaling factor.",
            ),
            DeclareLaunchArgument(
                "preview_max_width",
                default_value="",
                description="Override the preview max width used for live detection.",
            ),
            DeclareLaunchArgument(
                "preview_fast_check",
                default_value="",
                description="Override whether to use FAST_CHECK for live preview chessboard detection.",
            ),
            DeclareLaunchArgument(
                "annotated_jpeg_quality",
                default_value="",
                description="Override the annotated preview JPEG quality.",
            ),
            DeclareLaunchArgument(
                "input_qos_reliability",
                default_value="",
                description="Override the input QoS reliability: best_effort or reliable.",
            ),
            DeclareLaunchArgument(
                "input_qos_depth",
                default_value="",
                description="Override the input QoS queue depth.",
            ),
            DeclareLaunchArgument(
                "output_qos_reliability",
                default_value="",
                description="Override the annotated output QoS reliability: best_effort or reliable.",
            ),
            DeclareLaunchArgument(
                "output_qos_depth",
                default_value="",
                description="Override the annotated output QoS queue depth.",
            ),
            OpaqueFunction(function=_build_node),
        ]
    )
