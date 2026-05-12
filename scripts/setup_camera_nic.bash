#!/usr/bin/env bash

set -euo pipefail

interface="enp5s0"
host_cidr="192.168.1.10/24"
remove_cidr="192.168.1.0/24"
configure_ptp4l=true
dry_run=false

usage() {
  cat <<'USAGE'
Usage: scripts/setup_camera_nic.bash [options]

Prepare the PC camera NIC for the FLIR GigE/PTP launch.

Options:
  --interface IFACE       Camera NIC name. Default: enp5s0
  --host-cidr CIDR        PC host address to keep/add. Default: 192.168.1.10/24
  --remove-cidr CIDR      Bad PC alias to remove. Default: 192.168.1.0/24
  --skip-ptp4l-cap        Do not set ptp4l capabilities
  --dry-run               Print commands without running them
  -h, --help              Show this help

Example:
  scripts/setup_camera_nic.bash --interface enp5s0 --host-cidr 192.168.1.10/24
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interface)
      interface="${2:?missing value for --interface}"
      shift 2
      ;;
    --host-cidr)
      host_cidr="${2:?missing value for --host-cidr}"
      shift 2
      ;;
    --remove-cidr)
      remove_cidr="${2:?missing value for --remove-cidr}"
      shift 2
      ;;
    --skip-ptp4l-cap)
      configure_ptp4l=false
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v ip >/dev/null 2>&1; then
  echo "ip command not found." >&2
  exit 1
fi

if ! ip link show "$interface" >/dev/null 2>&1; then
  echo "Interface '$interface' not found." >&2
  exit 1
fi

run() {
  if [[ "$dry_run" == true ]]; then
    printf '+'
    printf ' %q' "$@"
    printf '\n'
    return
  fi

  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

has_cidr() {
  ip -o -4 addr show dev "$interface" | awk '{print $4}' | grep -Fxq "$1"
}

echo "[camera-nic] interface=${interface} host_cidr=${host_cidr} remove_cidr=${remove_cidr}"

run ip link set dev "$interface" up

if [[ -n "$remove_cidr" ]] && has_cidr "$remove_cidr"; then
  run ip addr del "$remove_cidr" dev "$interface"
fi

if ! has_cidr "$host_cidr"; then
  run ip addr add "$host_cidr" dev "$interface"
fi

if [[ "$configure_ptp4l" == true ]]; then
  if command -v ptp4l >/dev/null 2>&1; then
    run setcap cap_net_raw,cap_net_admin,cap_net_bind_service,cap_sys_time+ep "$(command -v ptp4l)"
  else
    echo "[camera-nic] ptp4l not found; install linuxptp to use PTP master launch." >&2
  fi
fi

ip -br addr show "$interface"
