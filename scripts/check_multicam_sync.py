#!/usr/bin/env python3
"""Measure how tightly the cameras actually expose together.

Each camera stamps a frame with its own PTP-disciplined clock and publishes it
as camera_timestamp_ns on image_raw/metadata. When PTP sync and the scheduled
action trigger both work, the eight cameras expose at the same PTP instant, so
those timestamps agree. This groups frames into trigger rounds and reports the
spread within each round, which is the synchronization error.

Do not judge sync from header.stamp: with use_camera_timestamp_in_header=false
that is host arrival time, which scatters by milliseconds no matter how well
the cameras are synchronized.

This needs PTP-synced camera clocks. With GPIO hardware triggering and PTP off
on the cameras, each camera_timestamp_ns is a separate counter and nothing here
lines up — use check_trigger_sync.py instead.

Usage:
  python3 scripts/check_multicam_sync.py
  python3 scripts/check_multicam_sync.py --duration 20 --cameras camera_center camera_rear
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from flir_spinnaker_camera.msg import FlirMetadata

DEFAULT_CAMERAS_FILE = "src/flir_spinnaker_camera/config/multicam_cameras.yaml"


def cameras_from_inventory(path: Path) -> list[str]:
    namespaces = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r'\s*namespace:\s*"?([^"\s]+)"?', line)
        if match:
            namespaces.append(match.group(1))
    return namespaces


class SyncChecker(Node):
    def __init__(self, cameras: list[str]):
        super().__init__("flir_multicam_sync_checker")
        self.stamps: dict[str, list[int]] = {camera: [] for camera in cameras}
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        for camera in cameras:
            self.create_subscription(
                FlirMetadata,
                f"/{camera}/image_raw/metadata",
                lambda msg, camera=camera: self.stamps[camera].append(msg.camera_timestamp_ns),
                qos,
            )


def median_period_ns(values: list[int]) -> float:
    if len(values) < 2:
        return 0.0
    ordered = sorted(values)
    return statistics.median(b - a for a, b in zip(ordered, ordered[1:]))


def group_into_rounds(stamps: dict[str, list[int]], window_ns: int) -> list[dict[str, int]]:
    """Match each reference frame with the nearest frame from every other camera.

    A camera that misses a trigger has no frame for that round. Its nearest frame
    is then one from the neighbouring round, a full period away, and counting that
    as sync error would report a drop as a huge offset. The window (half a period)
    excludes those instead of folding them into the statistics.
    """
    reference = max(stamps, key=lambda camera: len(stamps[camera]))
    rounds = []
    for anchor in stamps[reference]:
        round_stamps = {}
        for camera, values in stamps.items():
            nearest = min(values, key=lambda value: abs(value - anchor), default=None)
            if nearest is not None and abs(nearest - anchor) <= window_ns:
                round_stamps[camera] = nearest
        if len(round_stamps) > 1:
            rounds.append(round_stamps)
    return rounds


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0, help="Seconds to collect. Default: 10")
    parser.add_argument("--cameras", nargs="*", help="Camera namespaces. Default: read from the inventory YAML.")
    parser.add_argument("--cameras-file", default=DEFAULT_CAMERAS_FILE)
    # One trigger period at 12 Hz is 83 ms; half of that keeps rounds unambiguous.
    parser.add_argument("--window-ms", type=float, default=40.0, help="Max offset to call two frames the same round.")
    args = parser.parse_args()

    cameras = args.cameras or cameras_from_inventory(Path(args.cameras_file))
    if not cameras:
        print(f"No cameras found in {args.cameras_file}", file=sys.stderr)
        return 1

    rclpy.init()
    node = SyncChecker(cameras)
    print(f"Collecting {args.duration:.0f}s from {len(cameras)} cameras...")
    end_time = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    stamps = {camera: values for camera, values in node.stamps.items() if values}
    node.destroy_node()
    rclpy.shutdown()

    silent = [camera for camera in cameras if camera not in stamps]
    for camera in cameras:
        count = len(node.stamps[camera])
        period = median_period_ns(node.stamps[camera])
        rate = 1e9 / period if period > 0 else 0.0
        note = "  <-- SILENT" if count == 0 else ""
        print(f"  {camera:<22} {count:>4} frames  {rate:5.1f} Hz{note}")

    if len(stamps) < 2:
        print("\nNeed at least two publishing cameras to measure sync.", file=sys.stderr)
        return 1

    # Half a real trigger period is the widest a frame can be off and still
    # belong to this round rather than the next one.
    periods = [median_period_ns(values) for values in stamps.values() if len(values) > 1]
    window_ns = int(args.window_ms * 1e6)
    if periods:
        window_ns = min(window_ns, int(statistics.median(periods) / 2))
    print(f"\nRound matching window: {window_ns / 1e6:.1f} ms")

    rounds = group_into_rounds(stamps, window_ns)
    if not rounds:
        print("\nNo frames landed in a common window: the cameras are NOT synchronized.", file=sys.stderr)
        print("(Without PTP on the cameras their clocks share no epoch — for GPIO triggering use "
              "check_trigger_sync.py.)", file=sys.stderr)
        return 1

    full = [r for r in rounds if len(r) == len(stamps)]
    # Only rounds where every camera fired describe sync; a partial round means a
    # camera dropped that frame, which is a separate problem from clock offset.
    spreads_us = [(max(r.values()) - min(r.values())) / 1000.0 for r in (full or rounds)]

    print(f"\nMatched rounds: {len(rounds)} ({len(full)} with all {len(stamps)} cameras)")
    if len(full) < len(rounds):
        print(f"  {len(rounds) - len(full)} rounds were missing at least one camera (dropped frames).")
    print("Spread within a round (max - min camera_timestamp):")
    print(f"  median : {statistics.median(spreads_us):10.1f} us")
    print(f"  mean   : {statistics.mean(spreads_us):10.1f} us")
    print(f"  worst  : {max(spreads_us):10.1f} us")

    worst = max(spreads_us)
    if worst < 1000.0:
        print("\nSynchronized: every camera exposes within 1 ms of the others.")
    elif worst < 10000.0:
        print("\nLoosely synchronized (worst > 1 ms). PTP is holding but not tightly;")
        print("try ptp4l_timestamping:=hardware, or let PTP settle longer before measuring.")
    else:
        print("\nNOT synchronized (worst > 10 ms). The cameras are free-running:")
        print("check that PTP reached 'Slave' and that the action trigger is armed.")
    if silent:
        print(f"Cameras with no frames: {', '.join(silent)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
