#!/usr/bin/env python3
"""Verify GPIO hardware-triggered sync without a shared camera clock.

With the cameras triggered on GPIO and no PTP on the cameras, every
camera_timestamp_ns is that camera's own counter since power-on, so
check_multicam_sync.py (which compares those counters directly) cannot line
the cameras up. The trigger still hands every camera the same instants, though,
so the *difference* between two cameras' counters over matched frames must
follow a straight line: a constant epoch offset plus the two oscillators' rate
difference (a few ppm). How far the points stray from that line is the trigger
alignment error — sub-microsecond on the non-isolated input (Line3), a few
microseconds through the opto-isolated one (Line0).

Per camera, against a reference camera:
  lost      gaps in camera_frame_id — frames the camera never sent
  missing   trigger rounds with no frame from this camera on the host
  arrival   median host-arrival time difference. A few ms for a triggered
            camera (transfer differences); anywhere within half a period for a
            camera that is not taking the trigger (free-running at its own phase)
  jitter    residual std of the counter difference after removing offset and
            drift — the sync error
  drift     the fitted rate difference in ppm (oscillator tolerance)
For the rig: the spread of header.stamp within each round — what the recorded
data carries. With timestamp.mode=host that is host arrival time, so it shows
milliseconds even when the exposures are aligned to a microsecond; with
camera_latched + timestamp.trigger_grid_hz it should be 0.

Also works with PTP-synced camera clocks (PTP action triggering): the drift is
then ~0 ppm and the jitter is the PTP sync error.

Usage:
  python3 scripts/check_trigger_sync.py --duration 20
  python3 scripts/check_trigger_sync.py --duration 20 --json    # machine-readable (DM_clipGUI)
"""

from __future__ import annotations

import argparse
import bisect
import json
import statistics
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from flir_spinnaker_camera.msg import FlirMetadata

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_multicam_sync import DEFAULT_CAMERAS_FILE, cameras_from_inventory  # noqa: E402

NS_PER_S = 1_000_000_000
# camera_frame_id can restart (node relaunch); a jump this large is not lost frames
MAX_ID_JUMP = 100_000


class Frame:
    __slots__ = ("stamp", "frame_id", "header", "host", "exposure_us")

    def __init__(self, stamp: int, frame_id: int, header: int, host: int, exposure_us: float):
        self.stamp = stamp
        self.frame_id = frame_id
        self.header = header
        self.host = host
        self.exposure_us = exposure_us

    @classmethod
    def from_msg(cls, msg: FlirMetadata, host: int) -> "Frame":
        return cls(msg.camera_timestamp_ns, msg.camera_frame_id,
                   msg.header.stamp.sec * NS_PER_S + msg.header.stamp.nanosec, host, msg.exposure_time_us)


class Collector(Node):
    def __init__(self, cameras: list[str]):
        super().__init__("flir_trigger_sync_checker")
        self.frames: dict[str, list[Frame]] = {camera: [] for camera in cameras}
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        for camera in cameras:
            self.create_subscription(
                FlirMetadata,
                f"/{camera}/image_raw/metadata",
                lambda msg, camera=camera: self.frames[camera].append(Frame.from_msg(msg, time.time_ns())),
                qos,
            )


def median_period(stamps: list[int]) -> float:
    if len(stamps) < 2:
        return 0.0
    return statistics.median(b - a for a, b in zip(stamps, stamps[1:]))


def lost_frames(frame_ids: list[int]) -> int:
    lost = 0
    for a, b in zip(frame_ids, frame_ids[1:]):
        if 1 < b - a < MAX_ID_JUMP:
            lost += b - a - 1
    return lost


def match_rounds(frames: dict[str, list[Frame]], period: float) -> tuple[str, list[dict[str, Frame]]]:
    """(reference camera, [{camera: frame}]) — one round per reference frame.

    The counters share no epoch, so each camera is first moved onto the host
    clock by its median (host arrival - counter). Frames of different cameras
    then pair up by nearest mapped time within half a trigger period.
    """
    reference = max(frames, key=lambda camera: len(frames[camera]))
    mapped = {}
    for camera, values in frames.items():
        epoch = statistics.median(f.host - f.stamp for f in values)
        mapped[camera] = ([f.stamp + epoch for f in values], values)
    window = period / 2
    rounds = []
    ref_times, ref_frames = mapped[reference]
    for anchor, ref_frame in zip(ref_times, ref_frames):
        round_frames = {reference: ref_frame}
        for camera, (times, values) in mapped.items():
            if camera == reference:
                continue
            i = bisect.bisect_left(times, anchor)
            best = min((j for j in (i - 1, i) if 0 <= j < len(times)),
                       key=lambda j: abs(times[j] - anchor), default=None)
            if best is not None and abs(times[best] - anchor) <= window:
                round_frames[camera] = values[best]
        rounds.append(round_frames)
    return reference, rounds


def fit_pair(pairs: list[tuple[Frame, Frame]], period: float) -> dict:
    """Line through (reference counter, camera counter - reference counter).

    Integers are rebased before going to float so nanoseconds survive. A frame
    paired with the wrong trigger shows up a whole period off the line — those
    are dropped and counted rather than folded into the jitter.
    """
    ref0, cam0 = pairs[0][0].stamp, pairs[0][1].stamp
    x = np.array([ref.stamp - ref0 for ref, _ in pairs], dtype=np.float64)
    y = np.array([(cam.stamp - cam0) - (ref.stamp - ref0) for ref, cam in pairs], dtype=np.float64)
    keep = np.ones(len(x), dtype=bool)
    for _ in range(3):
        slope, intercept = np.polyfit(x[keep], y[keep], 1)
        residual = y - (slope * x + intercept)
        new_keep = np.abs(residual - np.median(residual[keep])) < period / 4
        if new_keep.sum() < 3 or (new_keep == keep).all():
            break
        keep = new_keep
    slope, intercept = np.polyfit(x[keep], y[keep], 1)
    residual = y[keep] - (slope * x[keep] + intercept)
    return {"jitter_us": float(np.std(residual)) / 1e3,
            "drift_ppm": float(slope) * 1e6,
            "mispaired": int((~keep).sum())}


def grid_phase(headers: list[int], grid_hz: float) -> dict:
    """Where header.stamp sits on the 1/grid_hz-second grid of the host clock.

    Only meaningful when header.stamp is the exposure time on a GNSS-synced host
    clock (timestamp.mode=camera_latched with trigger_grid_hz off): then a trigger
    pulse train locked to PPS lands at a fixed phase — ~0 if its edges are on the
    grid, a constant offset otherwise — and does not drift. A free-running pulse
    generator drifts by its frequency error (ppm).
    """
    period = NS_PER_S / grid_hz
    phases = np.array([(h % NS_PER_S) % period for h in headers], dtype=np.float64)
    angle = np.angle(np.mean(np.exp(2j * np.pi * phases / period)))
    center = angle * period / (2 * np.pi)
    unwrapped = center + (phases - center + period / 2) % period - period / 2
    t = np.array([h - headers[0] for h in headers], dtype=np.float64)
    drift = np.polyfit(t, unwrapped, 1)[0] if t[-1] > 0 else 0.0
    median = float(np.median(unwrapped))
    median = (median + period / 2) % period - period / 2
    return {"phase_ms": median / 1e6, "jitter_ms": float(np.std(unwrapped)) / 1e6,
            "drift_ppm": float(drift) * 1e6}


def analyze(frames: dict[str, list[Frame]], grid_hz: float = 0.0) -> dict:
    report: dict = {"cameras": {}, "reference": None, "rounds": None, "grid": None}
    publishing = {camera: values for camera, values in frames.items() if len(values) >= 2}
    periods = [median_period([f.stamp for f in values]) for values in publishing.values()]
    period = statistics.median(periods) if periods else 0.0
    report["period_ns"] = period
    report["rate_hz"] = NS_PER_S / period if period else 0.0

    for camera, values in frames.items():
        entry: dict = {"frames": len(values)}
        report["cameras"][camera] = entry
        if len(values) < 2:
            continue
        own = median_period([f.stamp for f in values])
        entry["rate_hz"] = NS_PER_S / own if own else 0.0
        entry["lost"] = lost_frames([f.frame_id for f in values])
        entry["exposure_us"] = statistics.median(f.exposure_us for f in values)

    # The trigger grid: given, or the measured rate if it is a whole number of Hz.
    if not grid_hz and report["rate_hz"] >= 1.0 and \
            abs(report["rate_hz"] - round(report["rate_hz"])) < 0.01 * round(report["rate_hz"]):
        grid_hz = float(round(report["rate_hz"]))
    if grid_hz and publishing:
        per_camera = {camera: grid_phase([f.header for f in values], grid_hz)
                      for camera, values in publishing.items()}
        for camera, phase in per_camera.items():
            report["cameras"][camera]["grid"] = phase
        report["grid"] = {
            "hz": grid_hz,
            "phase_ms": statistics.median(p["phase_ms"] for p in per_camera.values()),
            "jitter_ms": max(p["jitter_ms"] for p in per_camera.values()),
            "drift_ppm": statistics.median(p["drift_ppm"] for p in per_camera.values()),
        }

    if len(publishing) < 2 or not period:
        return report
    reference, rounds = match_rounds(publishing, period)
    report["reference"] = reference
    for camera in publishing:
        entry = report["cameras"][camera]
        paired = [(r[reference], r[camera]) for r in rounds if camera in r]
        entry["missing"] = len(rounds) - len(paired)
        if camera == reference or len(paired) < 3:
            continue
        entry["arrival_ms"] = statistics.median(cam.host - ref.host for ref, cam in paired) / 1e6
        entry.update(fit_pair(paired, period))

    full = [r for r in rounds if len(r) == len(publishing)]
    spreads = sorted((max(f.header for f in r.values()) - min(f.header for f in r.values())) / 1e6
                     for r in full)
    report["rounds"] = {
        "matched": len(rounds), "full": len(full), "cameras": len(publishing),
        "header_spread_median_ms": statistics.median(spreads) if spreads else None,
        "header_spread_p95_ms": spreads[min(len(spreads) - 1, int(0.95 * len(spreads)))] if spreads else None,
        "header_spread_worst_ms": spreads[-1] if spreads else None,
    }
    return report


def level(value: float | None, warn: float, fail: float) -> str:
    if value is None:
        return "-"
    return "OK" if value < warn else ("WARN" if value < fail else "FAIL")


def print_report(report: dict, args) -> None:
    print(f"\nTrigger period: {report['period_ns'] / 1e6:.3f} ms ({report['rate_hz']:.3f} Hz, camera clocks)")
    reference = report["reference"]
    print(f"\n  {'camera':<22} {'frames':>6} {'rate':>8} {'lost':>5} {'miss':>5} {'arrival':>9} "
          f"{'jitter':>10} {'drift':>10}")
    for camera, c in report["cameras"].items():
        if c["frames"] < 2:
            print(f"  {camera:<22} {c['frames']:>6}  <-- SILENT")
            continue
        if camera == reference:
            tail = "(reference)"
        elif c.get("jitter_us") is None:
            tail = "too few matched frames"
        else:
            tail = (f"{c['arrival_ms']:+7.2f}ms {c['jitter_us']:8.2f}us {c['drift_ppm']:+7.2f}ppm"
                    + (f"  ({c['mispaired']} off-line)" if c["mispaired"] else ""))
        print(f"  {camera:<22} {c['frames']:>6} {c['rate_hz']:6.2f}Hz {c['lost']:>5} "
              f"{c.get('missing', 0):>5} {tail}")

    grid = report.get("grid")
    if grid:
        print(f"\nheader.stamp on the {grid['hz']:g} Hz grid: phase {grid['phase_ms']:+.3f} ms, "
              f"jitter {grid['jitter_ms']:.3f} ms, drift {grid['drift_ppm']:+.2f} ppm")
        print("  (meaningful with timestamp.mode=camera_latched, trigger_grid_hz off, host clock synced to GNSS:"
              " phase ~0 and no drift = the pulses sit on the grid -> trigger_grid_hz can be turned on;"
              " constant phase = set trigger_grid_offset_ns; drift = the pulses are not locked to PPS)")

    rounds = report["rounds"]
    if not rounds:
        return
    print(f"\nRounds: {rounds['matched']} (reference frames), {rounds['full']} with all {rounds['cameras']} cameras")
    jitters = [c["jitter_us"] for c in report["cameras"].values() if c.get("jitter_us") is not None]
    if jitters:
        print(f"Trigger alignment: worst jitter {max(jitters):.2f} us -> "
              f"{level(max(jitters), args.jitter_warn_us, args.jitter_fail_us)} "
              f"(warn {args.jitter_warn_us:g} / fail {args.jitter_fail_us:g} us)")
    late = [camera for camera, c in report["cameras"].items()
            if c.get("arrival_ms") is not None and abs(c["arrival_ms"]) >= args.arrival_warn_ms]
    if late:
        print(f"Arrival more than {args.arrival_warn_ms:g} ms off the reference: {', '.join(late)}")
        print("  A camera that is not taking the trigger sits at an arbitrary phase — check its trigger role.")
    if rounds["header_spread_median_ms"] is not None:
        print(f"header.stamp spread within a round: median {rounds['header_spread_median_ms']:.2f} ms, "
              f"p95 {rounds['header_spread_p95_ms']:.2f} ms, worst {rounds['header_spread_worst_ms']:.2f} ms")
        if rounds["header_spread_worst_ms"] < 0.1:
            print("  header.stamp agrees across cameras: it is the exposure time (timestamp.mode=camera_latched + grid).")
        else:
            print("  The exposures are aligned to the jitter above; header.stamp is not. With timestamp.mode=host it is"
                  " host arrival time — use camera_latched (and timestamp.trigger_grid_hz).")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=20.0, help="Seconds to collect. Default: 20")
    parser.add_argument("--cameras", nargs="*", help="Camera namespaces. Default: read from the inventory YAML.")
    parser.add_argument("--cameras-file", default=DEFAULT_CAMERAS_FILE)
    parser.add_argument("--jitter-warn-us", type=float, default=10.0)
    parser.add_argument("--jitter-fail-us", type=float, default=1000.0)
    parser.add_argument("--arrival-warn-ms", type=float, default=10.0)
    parser.add_argument("--grid-hz", type=float, default=0.0,
                        help="Trigger grid for the header.stamp phase. Default: the measured rate if whole Hz.")
    parser.add_argument("--json", action="store_true", help="Print one JSON object on stdout, progress on stderr.")
    args = parser.parse_args()

    log = sys.stderr if args.json else sys.stdout
    cameras = args.cameras or cameras_from_inventory(Path(args.cameras_file))
    if not cameras:
        message = f"No cameras found in {args.cameras_file}"
        print(json.dumps({"error": message}) if args.json else message, file=sys.stdout if args.json else sys.stderr)
        return 1

    rclpy.init()
    node = Collector(cameras)
    print(f"Collecting {args.duration:.0f}s from {len(cameras)} cameras...", file=log, flush=True)
    end_time = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)
    frames = {camera: list(values) for camera, values in node.frames.items()}
    node.destroy_node()
    rclpy.shutdown()

    report = analyze(frames, args.grid_hz)
    report["duration_s"] = args.duration
    publishing = sum(1 for values in frames.values() if len(values) >= 2)
    if publishing < 2:
        report["error"] = ("no metadata from any camera (rig running? publish_metadata on?)" if not publishing
                           else "only one camera is publishing — need two to compare")

    if args.json:
        print(json.dumps(report))
    else:
        print_report(report, args)
        if report.get("error"):
            print(f"\n{report['error']}", file=sys.stderr)
    return 1 if report.get("error") else 0


if __name__ == "__main__":
    sys.exit(main())
