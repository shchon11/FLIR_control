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
  elif [[ -f "/opt/ros/noetic/setup.${_flir_env_setup_ext}" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/noetic/setup.${_flir_env_setup_ext}"
  elif [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
  elif [[ -f "/opt/ros/humble/setup.${_flir_env_setup_ext}" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/humble/setup.${_flir_env_setup_ext}"
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  else
    echo "[flir_env] ROS setup script not found. Set FLIR_ROS_DISTRO=noetic." >&2
    return 1
  fi
  unset _flir_env_requested_distro
fi

if [[ -z "${SPINNAKER_ROOT:-}" ]]; then
  for _flir_spinnaker_candidate in "${FLIR_ROS_WS}"/.spinnaker-*/opt/spinnaker /opt/spinnaker; do
    if [[ -d "${_flir_spinnaker_candidate}" ]]; then
      export SPINNAKER_ROOT="$(cd "${_flir_spinnaker_candidate}" && pwd)"
      break
    fi
  done
  unset _flir_spinnaker_candidate
fi

if [[ -n "${SPINNAKER_ROOT:-}" && -d "${SPINNAKER_ROOT}/lib" ]]; then
  if [[ "${LD_LIBRARY_PATH:-}" != "${SPINNAKER_ROOT}/lib"* ]]; then
    export LD_LIBRARY_PATH="${SPINNAKER_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
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
  echo "[flir_env] workspace setup not found yet. Build with catkin build for ROS 1 Noetic." >&2
fi

if [[ -n "${SPINNAKER_ROOT:-}" && -d "${SPINNAKER_ROOT}/lib" ]]; then
  if [[ "${LD_LIBRARY_PATH:-}" != "${SPINNAKER_ROOT}/lib"* ]]; then
    export LD_LIBRARY_PATH="${SPINNAKER_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
fi

if [[ "${ROS_VERSION:-}" == "1" ]]; then
  if [[ -z "${ROS_MASTER_URI:-}" ]]; then
    export ROS_MASTER_URI="http://127.0.0.1:11311"
  fi

  if [[ -n "${ROS_IP:-}" && "${FLIR_ROS_KEEP_NETWORK_ENV:-}" != "1" && "${ROS_IP}" != "127.0.0.1" ]]; then
    _flir_env_ros_ip_is_local=false
    if command -v ip >/dev/null 2>&1; then
      while IFS= read -r _flir_env_local_ip; do
        if [[ "${ROS_IP}" == "${_flir_env_local_ip}" ]]; then
          _flir_env_ros_ip_is_local=true
          break
        fi
      done < <(ip -o -4 addr show | awk '{split($4, address, "/"); print address[1]}')
    fi

    if [[ "${_flir_env_ros_ip_is_local}" != "true" ]]; then
      echo "[flir_env] ROS_IP=${ROS_IP} is not assigned to this host. Using local ROS master at 127.0.0.1." >&2
      unset ROS_IP
      unset ROS_HOSTNAME
      export ROS_MASTER_URI="http://127.0.0.1:11311"
    fi
    unset _flir_env_ros_ip_is_local
    unset _flir_env_local_ip
  fi
fi

if [[ -z "${SPINNAKER_GENTL64_CTI:-}" && -f "${SPINNAKER_ROOT:-}/lib/spinnaker-gentl/Spinnaker_GenTL.cti" ]]; then
  export SPINNAKER_GENTL64_CTI="${SPINNAKER_ROOT}/lib/spinnaker-gentl/Spinnaker_GenTL.cti"
fi

if [[ -z "${SPINNAKER_ROOT:-}" ]]; then
  echo "[flir_env] SPINNAKER_ROOT is not set. Export it if your Spinnaker SDK is installed outside the default search paths." >&2
fi

unset _flir_env_script_dir
unset _flir_env_script_path
unset _flir_env_setup_ext
