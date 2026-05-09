#!/usr/bin/env python3

import argparse
import subprocess
import sys
from pathlib import Path


def strip_quotes(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


def apply_key_value(camera: dict, text: str) -> None:
    if ":" not in text:
        return
    key, value = text.split(":", 1)
    key = key.strip()
    value = strip_quotes(value.strip())
    if key:
        camera[key] = value


def parse_cameras_file(path: Path) -> list:
    cameras = []
    current = None
    in_cameras = False

    with path.open("r", encoding="utf-8") as stream:
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
                apply_key_value(current, stripped[2:].strip())
                continue
            if current is not None:
                apply_key_value(current, stripped)

    if current is not None:
        cameras.append(current)

    for index, camera in enumerate(cameras):
        camera.setdefault("name", f"camera{index}")
        camera.setdefault("namespace", camera["name"])
    return cameras


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Set the same ROS parameter on every configured FLIR camera node."
    )
    parser.add_argument("parameter", help="Parameter name, e.g. camera.ExposureAuto")
    parser.add_argument("value", help="Parameter value passed to ros2 param set")
    parser.add_argument(
        "--cameras-file",
        default="src/flir_spinnaker_camera/config/multicam_cameras.yaml",
        help="Camera inventory YAML.",
    )
    parser.add_argument(
        "--node-name",
        default="flir_camera",
        help="Camera node name inside each namespace.",
    )
    args = parser.parse_args()

    cameras = parse_cameras_file(Path(args.cameras_file))
    if not cameras:
        print(f"No cameras found in {args.cameras_file}", file=sys.stderr)
        return 1

    failures = []
    for camera in cameras:
        namespace = camera["namespace"].strip("/")
        node = f"/{namespace}/{args.node_name}" if namespace else f"/{args.node_name}"
        command = ["ros2", "param", "set", node, args.parameter, args.value]
        print(" ".join(command))
        result = subprocess.run(command, check=False)
        if result.returncode != 0:
            failures.append((camera["name"], node, result.returncode))

    if failures:
        for name, node, returncode in failures:
            print(f"Failed on {name} ({node}), exit={returncode}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
