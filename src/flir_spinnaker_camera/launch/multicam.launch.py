from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import ipaddress
import os
import shutil
import subprocess
import yaml


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Expected a boolean launch value, got: {value}")


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


def _append_optional_cli_arg(command: list, option_name: str, value: str) -> None:
    value = value.strip()
    if value:
        command.extend([option_name, value])


def _run_camera_inventory_update(context, cameras_file: str) -> None:
    if not _parse_bool(LaunchConfiguration("auto_update_cameras_file").perform(context)):
        return

    command = [
        "ros2",
        "run",
        "flir_spinnaker_camera",
        "flir_multicam_inventory_tool",
        "--output",
        cameras_file,
    ]
    _append_optional_cli_arg(
        command,
        "--force-ip-base",
        LaunchConfiguration("auto_update_force_ip_base").perform(context),
    )
    _append_optional_cli_arg(
        command,
        "--force-ip-subnet-mask",
        LaunchConfiguration("auto_update_force_ip_subnet_mask").perform(context),
    )
    _append_optional_cli_arg(
        command,
        "--force-ip-gateway",
        LaunchConfiguration("auto_update_force_ip_gateway").perform(context),
    )
    _append_optional_cli_arg(
        command,
        "--new-ptp-action-role",
        LaunchConfiguration("auto_update_new_ptp_action_role").perform(context),
    )
    _append_optional_cli_arg(
        command,
        "--new-hardware-trigger-role",
        LaunchConfiguration("auto_update_new_hardware_trigger_role").perform(context),
    )

    if _parse_bool(LaunchConfiguration("auto_update_first_camera_ptp_sender").perform(context)):
        command.append("--first-camera-ptp-sender")
    if _parse_bool(LaunchConfiguration("auto_update_drop_missing").perform(context)):
        command.append("--drop-missing")
    if _parse_bool(LaunchConfiguration("auto_apply_force_ip").perform(context)):
        command.append("--apply-force-ip")
        command.extend(
            [
                "--force-ip-wait-after-ms",
                LaunchConfiguration("auto_force_ip_wait_after_ms").perform(context),
                "--force-ip-rediscovery-timeout-ms",
                LaunchConfiguration("auto_force_ip_rediscovery_timeout_ms").perform(context),
                "--force-ip-max-attempts",
                LaunchConfiguration("auto_force_ip_max_attempts").perform(context),
            ]
        )

    result = subprocess.run(command, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            "Failed to auto-update camera inventory YAML. "
            f"Command exited with code {result.returncode}: {' '.join(command)}"
        )


def _ptp4l_timestamping_args(mode: str) -> list:
    normalized = mode.strip().lower()
    if normalized in {"", "default"}:
        return []
    if normalized in {"software", "sw"}:
        return ["-S"]
    if normalized in {"hardware", "hw"}:
        return ["-H"]
    if normalized in {"legacy", "legacy-hardware"}:
        return ["-L"]
    raise ValueError(f"Unsupported ptp4l_timestamping value: {mode}")


def _validate_interface_address(interface_name: str) -> None:
    result = subprocess.run(
        ["ip", "-o", "-4", "addr", "show", "dev", interface_name],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Could not inspect IPv4 address for interface '{interface_name}'.")

    invalid_host_addresses = []
    valid_host_addresses = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if "inet" not in parts:
            continue
        cidr = parts[parts.index("inet") + 1]
        interface = ipaddress.ip_interface(cidr)
        if interface.network.prefixlen < 31 and interface.ip in {
            interface.network.network_address,
            interface.network.broadcast_address,
        }:
            invalid_host_addresses.append(interface)
            continue
        valid_host_addresses.append(interface)

    if invalid_host_addresses:
        first = invalid_host_addresses[0]
        host_hint = str(valid_host_addresses[0]) if valid_host_addresses else (
            f"{first.network.network_address + 10}/{first.network.prefixlen}"
        )
        raise RuntimeError(
            f"Interface '{interface_name}' has unusable PC NIC address(es): "
            f"{', '.join(str(address) for address in invalid_host_addresses)}. "
            "These are addresses assigned to the PC NIC, not the switch management address. "
            "They create duplicate GEV interfaces and can confuse Spinnaker. Keep the switch "
            f"as-is, remove the bad PC alias with: sudo ip addr del {first} dev {interface_name}. "
            f"Then keep/add a PC host IP such as {host_hint}."
        )

    if valid_host_addresses:
        return

    raise RuntimeError(
        f"Interface '{interface_name}' has no IPv4 address. Prepare the camera NIC with: "
        f"scripts/setup_camera_nic.bash --interface {interface_name} --host-cidr 192.168.1.10/24"
    )


def _validate_ptp4l_capabilities(ptp4l_binary: str) -> str:
    ptp4l_path = shutil.which(ptp4l_binary)
    if ptp4l_path is None:
        raise RuntimeError(
            f"ptp4l_binary '{ptp4l_binary}' was not found. Install linuxptp or set ptp4l_binary."
        )
    if os.geteuid() == 0:
        return ptp4l_path

    getcap_binary = shutil.which("getcap")
    if getcap_binary is None:
        return ptp4l_path

    result = subprocess.run(
        [getcap_binary, ptp4l_path],
        check=False,
        capture_output=True,
        text=True,
    )
    capabilities = result.stdout.strip()
    required = {"cap_net_raw", "cap_net_admin", "cap_net_bind_service", "cap_sys_time"}
    granted = {
        token.split("=", 1)[0]
        for token in capabilities.replace(",", " ").split()
        if token.startswith("cap_")
    }
    if not required.issubset(granted):
        raise RuntimeError(
            "ptp4l needs elevated network/time capabilities when launched without sudo. Run:\n"
            f"  sudo setcap cap_net_raw,cap_net_admin,cap_net_bind_service,cap_sys_time+ep {ptp4l_path}\n"
            "or start ptp4l separately with sudo and omit ptp_master_interface from this launch."
        )

    return ptp4l_path


def _hardware_trigger_role(camera: dict) -> str:
    return camera.get("hardware_trigger_role", camera.get("trigger_role", "")).strip()


def _ptp_action_role(camera: dict) -> str:
    return camera.get("ptp_action_role", camera.get("action_role", "")).strip()


def _is_master(camera: dict) -> bool:
    normalized = _hardware_trigger_role(camera).replace("_", "").replace("-", "").lower()
    return normalized in {"master", "bfsmaster"}


def _is_ptp_action_sender(camera: dict) -> bool:
    normalized = _ptp_action_role(camera).replace("_", "").replace("-", "").lower()
    return normalized in {"sender", "send", "master"}


def _camera_parameter_overrides(
    camera: dict,
    camera_info_yaml_path: str,
    force_ip_in_camera_nodes: bool,
) -> dict:
    overrides = {
        "camera_serial": camera["serial"],
        "frame_id": camera["frame_id"],
        "camera_info.yaml_path": camera_info_yaml_path,
    }
    trigger_role = _hardware_trigger_role(camera)
    if trigger_role:
        overrides["hardware_trigger.role"] = trigger_role
    ptp_action_role = _ptp_action_role(camera)
    if ptp_action_role:
        overrides["ptp_action.role"] = ptp_action_role

    force_ip_address = camera.get("force_ip_address", "").strip()
    if force_ip_address and force_ip_in_camera_nodes:
        overrides["network.force_ip.enable"] = True
        overrides["network.force_ip.address"] = force_ip_address
    elif force_ip_address:
        overrides["network.force_ip.enable"] = False

    force_ip_enable = camera.get("force_ip_enable", "").strip()
    if force_ip_enable and force_ip_in_camera_nodes:
        overrides["network.force_ip.enable"] = _parse_bool(force_ip_enable)

    for camera_key, parameter_name in (
        ("force_ip_subnet_mask", "network.force_ip.subnet_mask"),
        ("force_ip_gateway", "network.force_ip.gateway"),
    ):
        value = camera.get(camera_key, "").strip()
        if value:
            overrides[parameter_name] = value

    for camera_key, parameter_name in (
        ("force_ip_only_if_link_local", "network.force_ip.only_if_link_local"),
    ):
        value = camera.get(camera_key, "").strip()
        if value:
            overrides[parameter_name] = _parse_bool(value)

    force_ip_wait_after_ms = camera.get("force_ip_wait_after_ms", "").strip()
    if force_ip_wait_after_ms:
        overrides["network.force_ip.wait_after_ms"] = int(force_ip_wait_after_ms)

    force_ip_rediscovery_timeout_ms = camera.get("force_ip_rediscovery_timeout_ms", "").strip()
    if force_ip_rediscovery_timeout_ms:
        overrides["network.force_ip.rediscovery_timeout_ms"] = int(force_ip_rediscovery_timeout_ms)

    return overrides


def _build_camera_node(
    camera: dict,
    shared_parameters: dict,
    camera_info_yaml_path: str,
    force_ip_in_camera_nodes: bool,
) -> Node:
    return Node(
        package="flir_spinnaker_camera",
        executable="flir_spinnaker_camera_node",
        name="flir_camera",
        namespace=camera["namespace"],
        output="screen",
        parameters=[
            shared_parameters,
            _camera_parameter_overrides(camera, camera_info_yaml_path, force_ip_in_camera_nodes),
        ],
    )


def _build_camera_nodes(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    cameras_file = LaunchConfiguration("cameras_file").perform(context)
    camera_info_yaml_path = LaunchConfiguration("camera_info_yaml_path").perform(context)
    master_start_delay = float(LaunchConfiguration("hardware_trigger_master_start_delay").perform(context))
    ptp_action_sender_start_delay = float(
        LaunchConfiguration("ptp_action_sender_start_delay").perform(context)
    )
    ptp_master_interface = LaunchConfiguration("ptp_master_interface").perform(context)
    force_ip_in_camera_nodes = _parse_bool(
        LaunchConfiguration("force_ip_in_camera_nodes").perform(context)
    )
    publish_extrinsics_tf = _parse_bool(LaunchConfiguration("publish_extrinsics_tf").perform(context))
    extrinsics_yaml_path = LaunchConfiguration("extrinsics_yaml_path").perform(context)
    extrinsics_tf_frame_prefix = LaunchConfiguration("extrinsics_tf_frame_prefix").perform(context)

    ptp4l_process = None
    if ptp_master_interface:
        _validate_interface_address(ptp_master_interface)
        ptp4l_binary = LaunchConfiguration("ptp4l_binary").perform(context)
        ptp4l_path = _validate_ptp4l_capabilities(ptp4l_binary)
        timestamping_args = _ptp4l_timestamping_args(
            LaunchConfiguration("ptp4l_timestamping").perform(context)
        )
        uds_address = LaunchConfiguration("ptp4l_uds_address").perform(context)
        uds_args = ["--uds_address", uds_address] if uds_address else []
        ptp4l_process = ExecuteProcess(
            cmd=[
                ptp4l_path,
                "-i",
                ptp_master_interface,
                *timestamping_args,
                *uds_args,
                "-m",
            ],
            output="screen",
        )

    _run_camera_inventory_update(context, cameras_file)
    cameras = _parse_cameras_file(cameras_file)
    shared_parameters = _load_ros_parameters(params_file, "flir_camera")

    nodes = []
    if ptp4l_process is not None:
        nodes.append(ptp4l_process)

    if publish_extrinsics_tf:
        nodes.append(
            Node(
                package="flir_spinnaker_camera",
                executable="flir_camera_extrinsics_tf_node",
                name="flir_camera_extrinsics_tf",
                output="screen",
                parameters=[
                    {
                        "extrinsics_yaml_path": extrinsics_yaml_path,
                        "camera_serials": [camera["serial"] for camera in cameras],
                        "frame_prefix": extrinsics_tf_frame_prefix,
                    }
                ],
            )
        )

    master_nodes = []
    ptp_action_sender_nodes = []
    for camera in cameras:
        node = _build_camera_node(
            camera,
            shared_parameters,
            camera_info_yaml_path,
            force_ip_in_camera_nodes,
        )
        if _is_ptp_action_sender(camera):
            ptp_action_sender_nodes.append(node)
        elif _is_master(camera):
            master_nodes.append(node)
        else:
            nodes.append(node)

    if master_nodes and master_start_delay > 0.0:
        nodes.append(TimerAction(period=master_start_delay, actions=master_nodes))
    else:
        nodes.extend(master_nodes)

    if ptp_action_sender_nodes and ptp_action_sender_start_delay > 0.0:
        nodes.append(TimerAction(period=ptp_action_sender_start_delay, actions=ptp_action_sender_nodes))
    else:
        nodes.extend(ptp_action_sender_nodes)

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
            DeclareLaunchArgument(
                "auto_update_cameras_file",
                default_value="true",
                description="Detect connected Spinnaker cameras and merge them into cameras_file before launching.",
            ),
            DeclareLaunchArgument(
                "auto_update_force_ip_base",
                default_value="192.168.1.206",
                description="Optional first IPv4 address used to assign sequential force_ip_address entries.",
            ),
            DeclareLaunchArgument(
                "auto_update_force_ip_subnet_mask",
                default_value="255.255.255.0",
                description="Subnet mask written with auto-assigned ForceIP entries.",
            ),
            DeclareLaunchArgument(
                "auto_update_force_ip_gateway",
                default_value="0.0.0.0",
                description="Gateway written with auto-assigned ForceIP entries.",
            ),
            DeclareLaunchArgument(
                "auto_update_new_ptp_action_role",
                default_value="receiver",
                description="ptp_action_role assigned to newly discovered cameras.",
            ),
            DeclareLaunchArgument(
                "auto_update_new_hardware_trigger_role",
                default_value="none",
                description="hardware_trigger_role assigned to newly discovered cameras.",
            ),
            DeclareLaunchArgument(
                "auto_update_first_camera_ptp_sender",
                default_value="true",
                description="When no PTP sender exists, set the first camera to ptp_action_role=sender.",
            ),
            DeclareLaunchArgument(
                "auto_update_drop_missing",
                default_value="false",
                description="Remove cameras_file entries that are not currently detected.",
            ),
            DeclareLaunchArgument(
                "auto_apply_force_ip",
                default_value="true",
                description="Apply inventory ForceIP entries sequentially before camera nodes start.",
            ),
            DeclareLaunchArgument(
                "auto_force_ip_wait_after_ms",
                default_value="1500",
                description="Milliseconds to wait after each launch-time ForceIP command.",
            ),
            DeclareLaunchArgument(
                "auto_force_ip_rediscovery_timeout_ms",
                default_value="8000",
                description="Max milliseconds to wait for each camera to reappear after ForceIP.",
            ),
            DeclareLaunchArgument(
                "auto_force_ip_max_attempts",
                default_value="3",
                description="Max launch-time ForceIP command attempts per camera.",
            ),
            DeclareLaunchArgument(
                "force_ip_in_camera_nodes",
                default_value="false",
                description="Allow individual camera nodes to run ForceIP. Off by default to avoid parallel ForceIP.",
            ),
            DeclareLaunchArgument(
                "hardware_trigger_master_start_delay",
                default_value="1.0",
                description="Seconds to delay hardware-trigger master nodes so slave nodes can arm first.",
            ),
            DeclareLaunchArgument(
                "ptp_action_sender_start_delay",
                default_value="1.0",
                description="Seconds to delay PTP action sender nodes so receiver nodes can arm first.",
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
                "ptp4l_timestamping",
                default_value="software",
                description="ptp4l timestamping mode: software, hardware, legacy, or default.",
            ),
            DeclareLaunchArgument(
                "ptp4l_uds_address",
                default_value="/tmp/ptp4l-flir",
                description="Writable ptp4l UNIX socket path for local management messages.",
            ),
            DeclareLaunchArgument(
                "publish_extrinsics_tf",
                default_value="true",
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
            OpaqueFunction(function=_build_camera_nodes),
        ]
    )
