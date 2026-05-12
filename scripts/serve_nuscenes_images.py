#!/usr/bin/env python3
"""Serve a timestamp-sorted image viewer for an image-only nuScenes export."""

from __future__ import annotations

import argparse
import html
import json
import mimetypes
import os
import re
import socket
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import quote, unquote, urlparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve a localhost gallery for nuScenes sample images sorted by timestamp."
    )
    parser.add_argument(
        "dataset",
        nargs="?",
        help="nuScenes export root. Defaults to the newest directory under ./nuscenes_export.",
    )
    parser.add_argument(
        "--version",
        help="nuScenes table directory. Defaults to the first v* directory in the dataset root.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind host. Default: 127.0.0.1.")
    parser.add_argument("--port", type=int, default=8000, help="Bind port. Default: 8000.")
    return parser.parse_args()


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[1]


def latest_dataset(root: Path) -> Path:
    export_root = root / "nuscenes_export"
    candidates = [
        path
        for path in export_root.iterdir()
        if path.is_dir() and any(child.is_dir() and child.name.startswith("v") for child in path.iterdir())
    ]
    if not candidates:
        raise FileNotFoundError(f"No nuScenes exports found under {export_root}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def resolve_dataset(input_path: str | None, root: Path) -> Path:
    dataset = Path(input_path).expanduser() if input_path else latest_dataset(root)
    if not dataset.is_absolute():
        dataset = (root / dataset).resolve()
    if not dataset.is_dir():
        raise FileNotFoundError(f"Dataset root does not exist: {dataset}")
    return dataset


def resolve_version(dataset: Path, requested: str | None) -> str:
    if requested:
        if not (dataset / requested).is_dir():
            raise FileNotFoundError(f"Version directory does not exist: {dataset / requested}")
        return requested

    versions = sorted(child.name for child in dataset.iterdir() if child.is_dir() and child.name.startswith("v"))
    if not versions:
        raise FileNotFoundError(f"No nuScenes version directory found under {dataset}")
    return versions[0]


def load_json(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, list):
        raise ValueError(f"Expected list JSON table: {path}")
    return payload


def load_conversion_report(dataset: Path) -> dict:
    path = dataset / "conversion_report.json"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    return payload if isinstance(payload, dict) else {}


def timestamp_to_iso(timestamp_us: int) -> str:
    return datetime.fromtimestamp(timestamp_us / 1_000_000, tz=timezone.utc).isoformat()


def channel_sort_key(channel: str) -> tuple[int, str]:
    upper = channel.upper()
    if upper.endswith("_RAW") or "_RAW_" in upper:
        kind_rank = 0
    elif upper.endswith("_RGB") or "_RGB_" in upper:
        kind_rank = 1
    else:
        kind_rank = 2
    base = re.sub(r"_(RAW|RGB)(_|$)", "_", upper).strip("_")
    return kind_rank, base


def build_index(dataset: Path, version: str) -> dict:
    table_root = dataset / version
    samples = sorted(load_json(table_root / "sample.json"), key=lambda row: row["timestamp"])
    sample_data = sorted(load_json(table_root / "sample_data.json"), key=lambda row: row["timestamp"])
    sensors = {row["token"]: row for row in load_json(table_root / "sensor.json")}
    calibrated_sensors = {
        row["token"]: row for row in load_json(table_root / "calibrated_sensor.json")
    }

    channel_by_calibrated_sensor = {}
    for token, row in calibrated_sensors.items():
        sensor = sensors.get(row["sensor_token"], {})
        channel_by_calibrated_sensor[token] = sensor.get("channel", "UNKNOWN")

    report = load_conversion_report(dataset)
    sample_data_by_sample: dict[str, list[dict]] = {}
    channels = set()
    for row in sample_data:
        channel = channel_by_calibrated_sensor.get(row["calibrated_sensor_token"], "UNKNOWN")
        channels.add(channel)
        image_path = dataset / row["filename"]
        cache_key = f"{image_path.stat().st_mtime_ns}-{image_path.stat().st_size}" if image_path.exists() else "missing"
        sample_data_by_sample.setdefault(row["sample_token"], []).append(
            {
                "token": row["token"],
                "timestamp": row["timestamp"],
                "timestamp_iso": timestamp_to_iso(row["timestamp"]),
                "channel": channel,
                "filename": row["filename"],
                "url": "/data/" + quote(row["filename"]) + "?v=" + cache_key,
                "width": row.get("width", 0),
                "height": row.get("height", 0),
            }
        )

    frames = []
    for sample_index, sample in enumerate(samples):
        images = sorted(sample_data_by_sample.get(sample["token"], []), key=lambda row: channel_sort_key(row["channel"]))
        frames.append(
            {
                "index": sample_index,
                "token": sample["token"],
                "timestamp": sample["timestamp"],
                "timestamp_iso": timestamp_to_iso(sample["timestamp"]),
                "images": images,
            }
        )

    return {
        "dataset": str(dataset),
        "version": version,
        "scene": load_json(table_root / "scene.json")[0] if (table_root / "scene.json").exists() else {},
        "channels": sorted(channels, key=channel_sort_key),
        "frames": frames,
        "image_count": len(sample_data),
        "undistorted_export": bool(report.get("undistorted_export", False)),
        "preserve_bayer_raw": bool(report.get("preserve_bayer_raw", False)),
    }


def render_page(index: dict) -> bytes:
    dataset_name = html.escape(Path(index["dataset"]).name)
    version = html.escape(index["version"])
    scene_name = html.escape(index.get("scene", {}).get("name", dataset_name))
    channels_json = json.dumps(index["channels"])
    frames_json = json.dumps(index["frames"])
    image_count = int(index["image_count"])
    sample_count = len(index["frames"])
    preserve_bayer_raw = bool(index.get("preserve_bayer_raw", False))
    body_class = "raw-bayer-view" if preserve_bayer_raw else ""
    mode_label = " · undistorted raw Bayer + RGB" if preserve_bayer_raw else ""

    page = f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{scene_name} image viewer</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f6f7f9;
      --panel: #ffffff;
      --ink: #18202a;
      --muted: #5d6875;
      --line: #d8dde5;
      --accent: #087a7a;
      --accent-weak: #d9eeee;
      --shadow: 0 1px 2px rgba(15, 23, 42, .08);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--ink);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      letter-spacing: 0;
    }}
    header {{
      position: sticky;
      top: 0;
      z-index: 10;
      border-bottom: 1px solid var(--line);
      background: rgba(246, 247, 249, .96);
      backdrop-filter: blur(10px);
    }}
    .bar {{
      display: grid;
      grid-template-columns: minmax(220px, 1fr) auto;
      gap: 16px;
      align-items: end;
      max-width: 1680px;
      margin: 0 auto;
      padding: 14px 18px;
    }}
    h1 {{
      margin: 0;
      font-size: 18px;
      line-height: 1.25;
      font-weight: 700;
    }}
    .meta {{
      margin-top: 4px;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.35;
    }}
    .controls {{
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 8px;
      min-width: 280px;
    }}
    label {{
      display: inline-flex;
      align-items: center;
      gap: 6px;
      min-height: 34px;
      padding: 0 10px;
      border: 1px solid var(--line);
      background: var(--panel);
      border-radius: 6px;
      font-size: 13px;
      color: var(--ink);
      box-shadow: var(--shadow);
      white-space: nowrap;
    }}
    input[type="checkbox"] {{
      accent-color: var(--accent);
    }}
    main {{
      max-width: 1680px;
      margin: 0 auto;
      padding: 18px;
    }}
    .viewer {{
      display: grid;
      gap: 12px;
      margin-bottom: 16px;
      padding: 12px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      box-shadow: var(--shadow);
    }}
    .picker {{
      margin-top: 12px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      box-shadow: var(--shadow);
      overflow: hidden;
    }}
    .picker summary {{
      min-height: 42px;
      padding: 11px 12px;
      cursor: pointer;
      color: var(--ink);
      font-size: 13px;
      font-weight: 700;
      user-select: none;
    }}
    .picker-body {{
      display: grid;
      gap: 10px;
      padding: 0 12px 12px;
      border-top: 1px solid var(--line);
    }}
    .picker select {{
      width: 100%;
      min-height: 260px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #ffffff;
      color: var(--ink);
      font-size: 13px;
      font-variant-numeric: tabular-nums;
    }}
    .viewer-top {{
      display: grid;
      grid-template-columns: 38px minmax(0, 1fr) 38px;
      gap: 10px;
      align-items: center;
    }}
    .nav {{
      width: 38px;
      height: 38px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #ffffff;
      color: var(--ink);
      font-size: 22px;
      line-height: 1;
      cursor: pointer;
      box-shadow: var(--shadow);
    }}
    .nav:hover {{
      border-color: var(--accent);
      color: var(--accent);
    }}
    .viewer-stamp {{
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 8px 12px;
      min-width: 0;
    }}
    .stage {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(420px, 1fr));
      gap: 10px;
      min-width: 0;
    }}
    .stage.raw-rgb {{
      grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
      align-items: start;
    }}
    .stage img {{
      max-height: 62vh;
    }}
    .stage.raw-rgb img {{
      max-height: 38vh;
    }}
    .timeline {{
      display: grid;
      gap: 12px;
    }}
    .frame {{
      display: grid;
      grid-template-columns: 220px minmax(0, 1fr);
      gap: 12px;
      align-items: start;
      padding: 12px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      box-shadow: var(--shadow);
    }}
    .frame.selected {{
      border-color: var(--accent);
      box-shadow: 0 0 0 2px rgba(8, 122, 122, .18);
    }}
    .stamp {{
      position: sticky;
      top: 78px;
      display: grid;
      gap: 6px;
      min-width: 0;
    }}
    .index {{
      width: max-content;
      padding: 2px 7px;
      border-radius: 999px;
      background: var(--accent-weak);
      color: #075c5c;
      font-size: 12px;
      font-weight: 700;
    }}
    time {{
      font-variant-numeric: tabular-nums;
      font-size: 13px;
      line-height: 1.35;
      overflow-wrap: anywhere;
    }}
    .ts {{
      color: var(--muted);
      font-size: 12px;
      font-variant-numeric: tabular-nums;
      overflow-wrap: anywhere;
    }}
    .images {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 10px;
      min-width: 0;
    }}
    figure {{
      margin: 0;
      border: 1px solid var(--line);
      border-radius: 6px;
      overflow: hidden;
      background: #111820;
      min-width: 0;
    }}
    img {{
      display: block;
      width: 100%;
      aspect-ratio: 16 / 10;
      object-fit: contain;
      background: #111820;
    }}
    figcaption {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      min-height: 34px;
      padding: 7px 9px;
      background: #ffffff;
      color: var(--ink);
      font-size: 12px;
      font-variant-numeric: tabular-nums;
    }}
    .channel {{
      font-weight: 700;
      color: #164b4b;
      white-space: nowrap;
    }}
    .missing-frame {{
      display: grid;
      place-items: center;
      min-height: 160px;
      border: 1px dashed var(--line);
      border-radius: 6px;
      background: #f1f4f4;
      color: var(--muted);
      font-size: 12px;
      font-weight: 700;
    }}
    .empty {{
      padding: 28px;
      border: 1px dashed var(--line);
      border-radius: 8px;
      background: var(--panel);
      color: var(--muted);
      text-align: center;
    }}
    @media (max-width: 760px) {{
      .bar {{
        grid-template-columns: 1fr;
        align-items: start;
      }}
      .controls {{
        justify-content: flex-start;
      }}
      .frame {{
        grid-template-columns: 1fr;
      }}
      .viewer-top {{
        grid-template-columns: 34px minmax(0, 1fr) 34px;
      }}
      .nav {{
        width: 34px;
        height: 34px;
      }}
      .stamp {{
        position: static;
      }}
      .images,
      .stage {{
        grid-template-columns: 1fr;
      }}
      .stage.raw-rgb {{
        grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
      }}
    }}
  </style>
</head>
<body class="{body_class}">
  <header>
    <div class="bar">
      <div>
        <h1>{scene_name}</h1>
        <div class="meta">{version} · {sample_count} timestamps · {image_count} images · timestamp ascending{mode_label}</div>
      </div>
      <div class="controls" id="controls"></div>
    </div>
  </header>
  <main>
    <section class="viewer" id="viewer" aria-label="selected timestamp images"></section>
    <details class="picker">
      <summary>사진 목록</summary>
      <div class="picker-body">
        <select id="framePicker" size="12" aria-label="timestamp image list"></select>
      </div>
    </details>
  </main>
  <script>
    const channels = {channels_json};
    const frames = {frames_json};
    const enabled = new Set(channels);
    const controls = document.getElementById('controls');
    const viewer = document.getElementById('viewer');
    const framePicker = document.getElementById('framePicker');
    let currentFrameIndex = frames.length > 0 ? frames[0].index : 0;

    function isRawChannel(channel) {{
      return /(^|_)RAW($|_)/.test(channel.toUpperCase());
    }}

    function isRgbChannel(channel) {{
      return /(^|_)RGB($|_)/.test(channel.toUpperCase());
    }}

    function hasRawRgbLayout(images) {{
      return images.some(image => isRawChannel(image.channel)) &&
        images.some(image => isRgbChannel(image.channel));
    }}

    function cameraKeyForChannel(channel) {{
      return channel.toUpperCase().replace(/_(RAW|RGB)(_|$).*$/, '');
    }}

    function compareCameraKeys(left, right) {{
      return left.localeCompare(right, undefined, {{ numeric: true, sensitivity: 'base' }});
    }}

    function renderImageFigure(image) {{
      return `
        <figure class="${{isRawChannel(image.channel) ? 'raw-frame' : 'rgb-frame'}}">
          <img class="${{isRawChannel(image.channel) ? 'raw-bayer-image' : ''}}" src="${{image.url}}" alt="${{image.channel}} ${{image.timestamp}}">
          <figcaption>
            <span class="channel">${{image.channel}}</span>
            <span>${{image.width}}x${{image.height}}</span>
          </figcaption>
        </figure>
      `;
    }}

    function renderMissingFrame(cameraKey, streamKind) {{
      const label = cameraKey.replace(/^CAM_/, '');
      return `
        <div class="missing-frame" aria-label="${{label}} ${{streamKind}} missing">
          <span>${{label}} ${{streamKind}}</span>
        </div>
      `;
    }}

    function renderRawRgbStage(images) {{
      const groups = new Map();
      const otherImages = [];

      for (const image of images) {{
        if (!isRawChannel(image.channel) && !isRgbChannel(image.channel)) {{
          otherImages.push(image);
          continue;
        }}

        const key = cameraKeyForChannel(image.channel);
        if (!groups.has(key)) groups.set(key, {{ key, raw: null, rgb: null }});
        const group = groups.get(key);
        if (isRawChannel(image.channel)) group.raw = image;
        else group.rgb = image;
      }}

      const pairedImages = Array.from(groups.values())
        .sort((left, right) => compareCameraKeys(left.key, right.key))
        .map(group => `
          ${{group.raw ? renderImageFigure(group.raw) : renderMissingFrame(group.key, 'RAW')}}
          ${{group.rgb ? renderImageFigure(group.rgb) : renderMissingFrame(group.key, 'RGB')}}
        `)
        .join('');

      return pairedImages + otherImages.map(renderImageFigure).join('');
    }}

    function getVisibleFrames() {{
      return frames
        .map(frame => ({{
          ...frame,
          images: frame.images.filter(image => enabled.has(image.channel))
        }}))
        .filter(frame => frame.images.length > 0);
    }}

    function renderControls() {{
      controls.innerHTML = '';
      for (const channel of channels) {{
        const label = document.createElement('label');
        const input = document.createElement('input');
        input.type = 'checkbox';
        input.checked = enabled.has(channel);
        input.addEventListener('change', () => {{
          if (input.checked) enabled.add(channel);
          else enabled.delete(channel);
          renderViewer();
          renderPicker();
        }});
        label.append(input, document.createTextNode(channel));
        controls.append(label);
      }}
    }}

    function renderPicker() {{
      const visibleFrames = getVisibleFrames();
      framePicker.innerHTML = visibleFrames.map(frame => {{
        const option = document.createElement('option');
        const channelsText = frame.images.map(image => image.channel).join(', ');
        option.value = String(frame.index);
        option.textContent = `#${{String(frame.index).padStart(3, '0')}} | ${{frame.timestamp_iso}} | ${{channelsText}}`;
        if (frame.index === currentFrameIndex) option.selected = true;
        return option.outerHTML;
      }}).join('');
    }}

    function syncPicker() {{
      framePicker.value = String(currentFrameIndex);
    }}

    function renderViewer() {{
      const visibleFrames = getVisibleFrames();

      if (visibleFrames.length === 0) {{
        viewer.innerHTML = '<div class="empty">선택된 카메라 이미지가 없습니다.</div>';
        return;
      }}

      const frame = visibleFrames.find(candidate => candidate.index === currentFrameIndex) || visibleFrames[0];
      currentFrameIndex = frame.index;
      const visiblePosition = visibleFrames.findIndex(candidate => candidate.index === frame.index) + 1;
      const rawRgbLayout = hasRawRgbLayout(frame.images);
      const imagesMarkup = rawRgbLayout
        ? renderRawRgbStage(frame.images)
        : frame.images.map(renderImageFigure).join('');

      viewer.innerHTML = `
        <div class="viewer-top">
          <button class="nav" type="button" id="prevFrame" aria-label="Previous timestamp">&lt;</button>
          <div class="viewer-stamp">
            <span class="index">#${{frame.index}}</span>
            <span class="ts">${{visiblePosition}} / ${{visibleFrames.length}}</span>
            <time datetime="${{frame.timestamp_iso}}">${{frame.timestamp_iso}}</time>
            <span class="ts">${{frame.timestamp}} us</span>
          </div>
          <button class="nav" type="button" id="nextFrame" aria-label="Next timestamp">&gt;</button>
        </div>
        <div class="stage ${{rawRgbLayout ? 'raw-rgb' : ''}}">
          ${{imagesMarkup}}
        </div>
      `;
      document.getElementById('prevFrame').addEventListener('click', () => moveFrame(-1, true));
      document.getElementById('nextFrame').addEventListener('click', () => moveFrame(1, true));
      syncPicker();
    }}

    function setCurrentFrame(frameIndex, shouldScroll) {{
      currentFrameIndex = frameIndex;
      renderViewer();
    }}

    function moveFrame(delta, shouldScroll) {{
      const visibleFrames = getVisibleFrames();
      if (visibleFrames.length === 0) return;
      let position = visibleFrames.findIndex(frame => frame.index === currentFrameIndex);
      if (position < 0) position = 0;
      const nextPosition = Math.min(Math.max(position + delta, 0), visibleFrames.length - 1);
      setCurrentFrame(visibleFrames[nextPosition].index, shouldScroll);
    }}

    document.addEventListener('keydown', event => {{
      if (event.defaultPrevented) return;
      if (event.key === 'ArrowRight' || event.key === 'ArrowDown') {{
        event.preventDefault();
        moveFrame(1, true);
      }} else if (event.key === 'ArrowLeft' || event.key === 'ArrowUp') {{
        event.preventDefault();
        moveFrame(-1, true);
      }} else if (event.key === 'Home') {{
        const visibleFrames = getVisibleFrames();
        if (visibleFrames.length > 0) {{
          event.preventDefault();
          setCurrentFrame(visibleFrames[0].index, true);
        }}
      }} else if (event.key === 'End') {{
        const visibleFrames = getVisibleFrames();
        if (visibleFrames.length > 0) {{
          event.preventDefault();
          setCurrentFrame(visibleFrames[visibleFrames.length - 1].index, true);
        }}
      }}
    }});

    framePicker.addEventListener('change', () => {{
      setCurrentFrame(Number(framePicker.value), false);
    }});

    renderControls();
    renderPicker();
    renderViewer();
  </script>
</body>
</html>
"""
    return page.encode("utf-8")


def safe_dataset_path(dataset: Path, relative_path: str) -> Path | None:
    decoded = unquote(relative_path).lstrip("/")
    candidate = (dataset / decoded).resolve()
    try:
        candidate.relative_to(dataset)
    except ValueError:
        return None
    return candidate


def make_handler(dataset: Path, version: str):
    class ViewerHandler(BaseHTTPRequestHandler):
        server_version = "NuScenesImageViewer/1.0"

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path in {"/", "/index.html"}:
                self.send_html()
                return
            if parsed.path == "/api/index":
                self.send_json()
                return
            if parsed.path.startswith("/data/"):
                self.send_file(parsed.path.removeprefix("/data/"))
                return
            self.send_error(404, "Not found")

        def log_message(self, format: str, *args) -> None:
            print(f"{self.address_string()} - {format % args}")

        def send_html(self) -> None:
            body = render_page(build_index(dataset, version))
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def send_json(self) -> None:
            body = json.dumps(build_index(dataset, version)).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def send_file(self, relative_path: str) -> None:
            path = safe_dataset_path(dataset, relative_path)
            if path is None or not path.is_file():
                self.send_error(404, "File not found")
                return
            content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            stat = path.stat()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(stat.st_size))
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.end_headers()
            with path.open("rb") as stream:
                while True:
                    chunk = stream.read(1024 * 256)
                    if not chunk:
                        break
                    self.wfile.write(chunk)

    return ViewerHandler


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def find_available_port(host: str, preferred_port: int) -> int:
    for port in range(preferred_port, preferred_port + 100):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind((host, port))
            except OSError:
                continue
            return port
    raise OSError(f"No available port found from {preferred_port} to {preferred_port + 99}")


def main() -> None:
    args = parse_args()
    root = workspace_root()
    dataset = resolve_dataset(args.dataset, root)
    version = resolve_version(dataset, args.version)
    port = find_available_port(args.host, args.port)
    os.chdir(dataset)
    server = ReusableThreadingHTTPServer((args.host, port), make_handler(dataset, version))
    url = f"http://{args.host}:{port}"
    print(f"Serving {dataset} ({version})")
    print(f"Open {url}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
