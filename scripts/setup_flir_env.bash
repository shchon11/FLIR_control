#!/usr/bin/env bash

if [[ -n "${BASH_VERSION:-}" && "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "source this file instead of executing it: source scripts/setup_flir_env.bash" >&2
  exit 1
fi

if [[ -n "${BASH_VERSION:-}" ]]; then
  _flir_env_script_path="${BASH_SOURCE[0]}"
elif [[ -n "${ZSH_VERSION:-}" ]]; then
  eval '_flir_env_script_path="${(%):-%x}"'
else
  _flir_env_script_path="${0}"
fi

_flir_env_script_dir="$(cd "$(dirname "${_flir_env_script_path}")" && pwd)"
export FLIR_ROS_WS="$(cd "${_flir_env_script_dir}/.." && pwd)"
_flir_env_setup_ext="bash"
if [[ -n "${ZSH_VERSION:-}" ]]; then
  _flir_env_setup_ext="zsh"
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  _flir_env_requested_distro="${FLIR_ROS_DISTRO:-}"
  if [[ -n "${_flir_env_requested_distro}" && -f "/opt/ros/${_flir_env_requested_distro}/setup.${_flir_env_setup_ext}" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${_flir_env_requested_distro}/setup.${_flir_env_setup_ext}"
  elif [[ -n "${_flir_env_requested_distro}" && -f "/opt/ros/${_flir_env_requested_distro}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${_flir_env_requested_distro}/setup.bash"
  elif [[ -f "/opt/ros/humble/setup.${_flir_env_setup_ext}" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/humble/setup.${_flir_env_setup_ext}"
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  elif [[ -f "/opt/ros/noetic/setup.${_flir_env_setup_ext}" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/noetic/setup.${_flir_env_setup_ext}"
  elif [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
  else
    echo "[flir_env] ROS setup script not found. Set FLIR_ROS_DISTRO=humble or FLIR_ROS_DISTRO=noetic." >&2
    return 1
  fi
  unset _flir_env_requested_distro
fi

if [[ -z "${SPINNAKER_ROOT:-}" && -d /opt/spinnaker ]]; then
  export SPINNAKER_ROOT=/opt/spinnaker
fi

if [[ "${ROS_VERSION:-}" == "1" && -f "${FLIR_ROS_WS}/devel/setup.${_flir_env_setup_ext}" ]]; then
  # shellcheck disable=SC1090
  source "${FLIR_ROS_WS}/devel/setup.${_flir_env_setup_ext}"
elif [[ "${ROS_VERSION:-}" == "1" && -f "${FLIR_ROS_WS}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${FLIR_ROS_WS}/devel/setup.bash"
elif [[ -f "${FLIR_ROS_WS}/install/setup.${_flir_env_setup_ext}" ]]; then
  # shellcheck disable=SC1091
  source "${FLIR_ROS_WS}/install/setup.${_flir_env_setup_ext}"
elif [[ -f "${FLIR_ROS_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${FLIR_ROS_WS}/install/setup.bash"
else
  echo "[flir_env] workspace setup not found yet. Build with colcon for ROS 2 or catkin_make for ROS 1." >&2
fi

if [[ -z "${SPINNAKER_ROOT:-}" ]]; then
  echo "[flir_env] SPINNAKER_ROOT is not set. Export it if your Spinnaker SDK is installed outside the default search paths." >&2
fi

unset _flir_env_script_dir
unset _flir_env_script_path
unset _flir_env_setup_ext
