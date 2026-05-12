from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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


def _build_camera_node(context):
    parameter_overrides = {}

    for name in (
        "camera_serial",
        "frame_id",
        "pixel_format",
        "rgb_compression_format",
        "publisher_qos_reliability",
        "camera_info_yaml_path",
        "hardware_trigger_role",
        "force_ip_address",
        "force_ip_subnet_mask",
        "force_ip_gateway",
        "ptp_action_role",
    ):
        value = _optional_override(context, name)
        if value is not None:
            if name == "camera_info_yaml_path":
                parameter_overrides["camera_info.yaml_path"] = value
            elif name == "hardware_trigger_role":
                parameter_overrides["hardware_trigger.role"] = value
            elif name == "force_ip_address":
                parameter_overrides["network.force_ip.address"] = value
                parameter_overrides["network.force_ip.enable"] = True
            elif name == "force_ip_subnet_mask":
                parameter_overrides["network.force_ip.subnet_mask"] = value
            elif name == "force_ip_gateway":
                parameter_overrides["network.force_ip.gateway"] = value
            elif name == "ptp_action_role":
                parameter_overrides["ptp_action.role"] = value
            else:
                parameter_overrides[name] = value

    for name in (
        "auto_pixel_format",
        "publish_camera_info",
        "publish_metadata",
        "publish_rgb_compressed",
        "force_ip_enable",
        "force_ip_only_if_link_local",
        "ptp_enable",
        "ptp_action_request_ack",
        "use_camera_timestamp_in_header",
    ):
        value = _optional_override(context, name, _parse_bool)
        if value is not None:
            if name == "force_ip_enable":
                parameter_overrides["network.force_ip.enable"] = value
            elif name == "force_ip_only_if_link_local":
                parameter_overrides["network.force_ip.only_if_link_local"] = value
            elif name == "ptp_enable":
                parameter_overrides["ptp.enable"] = value
            elif name == "ptp_action_request_ack":
                parameter_overrides["ptp_action.request_ack"] = value
            else:
                parameter_overrides[name] = value

    for name in (
        "rgb_jpeg_quality",
        "rgb_png_compression_level",
        "publisher_qos_depth",
        "force_ip_wait_after_ms",
        "ptp_action_expected_ack_count",
    ):
        value = _optional_override(context, name, int)
        if value is not None:
            if name == "force_ip_wait_after_ms":
                parameter_overrides["network.force_ip.wait_after_ms"] = value
            elif name == "ptp_action_expected_ack_count":
                parameter_overrides["ptp_action.expected_ack_count"] = value
            else:
                parameter_overrides[name] = value

    for name in ("ptp_action_rate_hz", "ptp_action_schedule_ahead_ms"):
        value = _optional_override(context, name, float)
        if value is not None:
            if name == "ptp_action_rate_hz":
                parameter_overrides["ptp_action.rate_hz"] = value
            elif name == "ptp_action_schedule_ahead_ms":
                parameter_overrides["ptp_action.schedule_ahead_ms"] = value

    actions = []
    publish_extrinsics_tf = _optional_override(context, "publish_extrinsics_tf", _parse_bool)
    if publish_extrinsics_tf:
        extrinsics_parameters = {
            "extrinsics_yaml_path": LaunchConfiguration("extrinsics_yaml_path").perform(context),
            "frame_prefix": LaunchConfiguration("extrinsics_tf_frame_prefix").perform(context),
        }
        camera_serial = LaunchConfiguration("camera_serial").perform(context)
        if camera_serial:
            extrinsics_parameters["camera_serials"] = [camera_serial]

        actions.append(
            Node(
                package="flir_spinnaker_camera",
                executable="flir_camera_extrinsics_tf_node",
                name="flir_camera_extrinsics_tf",
                output="screen",
                parameters=[extrinsics_parameters],
            )
        )

    ptp_master_interface = LaunchConfiguration("ptp_master_interface").perform(context)
    if ptp_master_interface:
        actions.append(
            ExecuteProcess(
                cmd=[
                    LaunchConfiguration("ptp4l_binary"),
                    "-i",
                    ptp_master_interface,
                    "-m",
                ],
                output="screen",
            )
        )

    actions.append(
        Node(
            package="flir_spinnaker_camera",
            executable="flir_spinnaker_camera_node",
            name="flir_camera",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                parameter_overrides,
            ],
        )
    )
    return actions


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("flir_spinnaker_camera"), "config", "flir_camera.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the FLIR camera node.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Path to the ROS 2 parameter file.",
            ),
            DeclareLaunchArgument(
                "camera_serial",
                default_value="",
                description="Optional FLIR camera serial number.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Optional frame_id override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "camera_info_yaml_path",
                default_value="",
                description="Optional path to a calibration YAML whose camera_info.* values override the defaults.",
            ),
            DeclareLaunchArgument(
                "hardware_trigger_role",
                default_value="",
                description="Optional BFS hardware trigger role override: none, master, or slave.",
            ),
            DeclareLaunchArgument(
                "force_ip_enable",
                default_value="",
                description="Optional GigE ForceIP enable override.",
            ),
            DeclareLaunchArgument(
                "force_ip_address",
                default_value="",
                description="Optional GigE ForceIP target address, e.g. 192.168.1.206.",
            ),
            DeclareLaunchArgument(
                "force_ip_subnet_mask",
                default_value="",
                description="Optional GigE ForceIP subnet mask, e.g. 255.255.255.0.",
            ),
            DeclareLaunchArgument(
                "force_ip_gateway",
                default_value="",
                description="Optional GigE ForceIP gateway, e.g. 0.0.0.0.",
            ),
            DeclareLaunchArgument(
                "force_ip_only_if_link_local",
                default_value="",
                description="Only force IP when the current camera address is 169.254.x.x.",
            ),
            DeclareLaunchArgument(
                "force_ip_wait_after_ms",
                default_value="",
                description="Milliseconds to wait after sending the ForceIP command before opening the camera.",
            ),
            DeclareLaunchArgument(
                "ptp_enable",
                default_value="",
                description="Optional PTP enable override. True sets GevIEEE1588 on the camera.",
            ),
            DeclareLaunchArgument(
                "ptp_master_interface",
                default_value="",
                description="Optional NIC name. When set, launch starts ptp4l as the PC PTP master.",
            ),
            DeclareLaunchArgument(
                "ptp4l_binary",
                default_value="ptp4l",
                description="ptp4l executable used when ptp_master_interface is set.",
            ),
            DeclareLaunchArgument(
                "publish_extrinsics_tf",
                default_value="false",
                description="Publish calibrated camera extrinsics from extrinsics_yaml_path to /tf_static.",
            ),
            DeclareLaunchArgument(
                "extrinsics_yaml_path",
                default_value="calibration/flir_camera_extrinsics.yaml",
                description="Path to calibration YAML with extrinsics_by_serial entries.",
            ),
            DeclareLaunchArgument(
                "extrinsics_tf_frame_prefix",
                default_value="",
                description="Optional prefix applied to parent and child frame IDs in published extrinsic TF.",
            ),
            DeclareLaunchArgument(
                "ptp_action_role",
                default_value="",
                description="Optional PTP action role override: none, receiver, or sender.",
            ),
            DeclareLaunchArgument(
                "ptp_action_rate_hz",
                default_value="",
                description="Optional scheduled action sender rate in Hz.",
            ),
            DeclareLaunchArgument(
                "ptp_action_schedule_ahead_ms",
                default_value="",
                description="Optional scheduled action lead time in milliseconds.",
            ),
            DeclareLaunchArgument(
                "ptp_action_request_ack",
                default_value="",
                description="Optional action command acknowledgement request override.",
            ),
            DeclareLaunchArgument(
                "ptp_action_expected_ack_count",
                default_value="",
                description="Optional expected acknowledgement count when requesting action command ACKs.",
            ),
            DeclareLaunchArgument(
                "publisher_qos_reliability",
                default_value="",
                description="Optional publisher QoS reliability override: reliable or best_effort.",
            ),
            DeclareLaunchArgument(
                "auto_pixel_format",
                default_value="",
                description="Optional auto_pixel_format override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "use_camera_timestamp_in_header",
                default_value="",
                description="Optional header timestamp source override. True uses camera timestamps when available.",
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="",
                description="Optional camera PixelFormat override such as BayerRG8, BayerRG12p, or BayerRG16.",
            ),
            DeclareLaunchArgument(
                "publish_camera_info",
                default_value="",
                description="Optional publish_camera_info override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "publish_metadata",
                default_value="",
                description="Optional publish_metadata override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "publish_rgb_compressed",
                default_value="",
                description="Optional publish_rgb_compressed override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "rgb_compression_format",
                default_value="",
                description="Optional compressed RGB format override: jpeg or png.",
            ),
            DeclareLaunchArgument(
                "rgb_jpeg_quality",
                default_value="",
                description="Optional JPEG quality override for /image_rgb/compressed.",
            ),
            DeclareLaunchArgument(
                "rgb_png_compression_level",
                default_value="",
                description="Optional PNG compression level override for /image_rgb/compressed.",
            ),
            DeclareLaunchArgument(
                "publisher_qos_depth",
                default_value="",
                description="Optional publisher QoS queue depth override.",
            ),
            OpaqueFunction(function=_build_camera_node),
        ]
    )
