#!/usr/bin/env bash
set -euo pipefail

binary="ptp4l"
interface=""
timestamping="software"
uds_address="/tmp/ptp4l-flir"
extra_args=()

while (($# > 0)); do
  case "$1" in
    --binary)
      binary="${2:?--binary requires a value}"
      shift 2
      ;;
    --interface|-i)
      interface="${2:?--interface requires a value}"
      shift 2
      ;;
    --timestamping)
      timestamping="${2:?--timestamping requires a value}"
      shift 2
      ;;
    --uds-address)
      uds_address="${2:?--uds-address requires a value}"
      shift 2
      ;;
    __name:=*|__log:=*|__ns:=*|__cwd:=*)
      shift
      ;;
    *)
      extra_args+=("$1")
      shift
      ;;
  esac
done

if [[ -z "$interface" ]]; then
  echo "ptp4l wrapper error: --interface is required." >&2
  exit 2
fi

ptp4l_path="$(command -v "$binary" || true)"
if [[ -z "$ptp4l_path" ]]; then
  echo "ptp4l wrapper error: '$binary' was not found. Install linuxptp or set ptp4l_binary." >&2
  exit 2
fi

if ! ip -o -4 addr show dev "$interface" >/dev/null 2>&1; then
  echo "ptp4l wrapper error: could not inspect interface '$interface'." >&2
  exit 2
fi

if [[ -z "$(ip -o -4 addr show dev "$interface")" ]]; then
  echo "ptp4l wrapper error: interface '$interface' has no IPv4 address." >&2
  exit 2
fi

if [[ "${EUID}" -ne 0 ]] && command -v getcap >/dev/null 2>&1; then
  capabilities="$(getcap "$ptp4l_path" 2>/dev/null || true)"
  missing=()
  for capability in cap_net_raw cap_net_admin cap_net_bind_service cap_sys_time; do
    if [[ "$capabilities" != *"$capability"* ]]; then
      missing+=("$capability")
    fi
  done

  if ((${#missing[@]} > 0)); then
    echo "ptp4l wrapper error: '$ptp4l_path' needs elevated capabilities for roslaunch." >&2
    echo "Run: sudo setcap cap_net_raw,cap_net_admin,cap_net_bind_service,cap_sys_time+ep $ptp4l_path" >&2
    echo "Or start ptp4l separately with sudo and leave ptp_master_interface empty." >&2
    exit 1
  fi
fi

timestamping_args=()
case "${timestamping,,}" in
  ""|"default")
    ;;
  "software"|"sw")
    timestamping_args=(-S)
    ;;
  "hardware"|"hw")
    timestamping_args=(-H)
    ;;
  "legacy"|"legacy-hardware")
    timestamping_args=(-L)
    ;;
  *)
    echo "ptp4l wrapper error: unsupported --timestamping '$timestamping'." >&2
    exit 2
    ;;
esac

cmd=("$ptp4l_path" -i "$interface" "${timestamping_args[@]}")
if [[ -n "$uds_address" ]]; then
  cmd+=(--uds_address "$uds_address")
fi
cmd+=(-m)
cmd+=("${extra_args[@]}")

exec "${cmd[@]}"
