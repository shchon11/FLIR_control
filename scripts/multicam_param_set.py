#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rospy
import yaml

from flir_spinnaker_camera.srv import SetCameraControl


def normalize_scope_and_name(parameter: str, explicit_scope: str) -> tuple[str, str]:
    if explicit_scope:
        return explicit_scope, parameter

    if "." in parameter:
        prefix, name = parameter.split(".", 1)
        normalized = prefix.strip().lower().replace("-", "_")
        if normalized in {"camera", "stream", "tl_device", "tldevice", "device"}:
            return normalized, name

    return "camera", parameter


def load_cameras(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}

    params = payload.get("flir_multicam", {}).get("ros__parameters", {})
    cameras = params.get("cameras", [])
    if not isinstance(cameras, list):
        raise RuntimeError(f"Invalid camera inventory in {path}: cameras must be a list")

    result = []
    for index, camera in enumerate(cameras):
        if not isinstance(camera, dict):
            continue
        item = dict(camera)
        item.setdefault("name", f"camera{index}")
        item.setdefault("namespace", item["name"])
        result.append(item)
    return result


def service_name_for(namespace: str, service_basename: str) -> str:
    namespace = str(namespace or "").strip("/")
    service_basename = service_basename.strip("/")
    return f"/{namespace}/{service_basename}" if namespace else f"/{service_basename}"


def call_control_service(service_name: str, scope: str, name: str, value: str, timeout: float) -> tuple[bool, str]:
    rospy.wait_for_service(service_name, timeout=timeout)
    proxy = rospy.ServiceProxy(service_name, SetCameraControl)
    response = proxy(scope=scope, name=name, value=value)
    detail = response.message
    if response.current_value:
        detail += f" current={response.current_value}"
    if response.applied_type:
        detail += f" type={response.applied_type}"
    return bool(response.success), detail


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Set one Spinnaker GenICam control on every configured Noetic FLIR camera node. "
            "Examples: camera.ExposureAuto Off, camera.ExposureTime 8000, stream.StreamBufferCountManual 64"
        )
    )
    parser.add_argument("parameter", help="Control name, optionally prefixed by camera., stream., or tl_device.")
    parser.add_argument("value", help="Value to write, or 'execute' for command nodes.")
    parser.add_argument(
        "--cameras-file",
        default="src/flir_spinnaker_camera/config/multicam_cameras.yaml",
        help="Camera inventory YAML.",
    )
    parser.add_argument(
        "--scope",
        default="",
        choices=["", "camera", "stream", "tl_device", "tldevice", "device"],
        help="Explicit control scope. Overrides any PARAMETER prefix.",
    )
    parser.add_argument(
        "--service-name",
        default="set_camera_control",
        help="Service basename advertised under each camera namespace.",
    )
    parser.add_argument(
        "--camera",
        action="append",
        default=[],
        help="Only target this camera name/namespace/serial. Can be repeated.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for each camera service.",
    )
    args = parser.parse_args()

    cameras = load_cameras(Path(args.cameras_file))
    if args.camera:
        requested = set(args.camera)
        cameras = [
            camera
            for camera in cameras
            if camera.get("name") in requested
            or camera.get("namespace") in requested
            or str(camera.get("serial", "")) in requested
        ]
    if not cameras:
        print("No matching cameras found.", file=sys.stderr)
        return 1

    scope, name = normalize_scope_and_name(args.parameter, args.scope)
    rospy.init_node("flir_multicam_param_set", anonymous=True, disable_signals=True)

    failures = []
    for camera in cameras:
        namespace = camera.get("namespace", camera.get("name", ""))
        service_name = service_name_for(namespace, args.service_name)
        label = camera.get("name", namespace or service_name)
        try:
            success, detail = call_control_service(service_name, scope, name, args.value, args.timeout)
        except Exception as exc:
            failures.append((label, service_name, str(exc)))
            print(f"[FAIL] {label} {service_name}: {exc}", file=sys.stderr)
            continue

        prefix = "OK" if success else "FAIL"
        print(f"[{prefix}] {label} {scope}.{name}={args.value}: {detail}")
        if not success:
            failures.append((label, service_name, detail))

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
