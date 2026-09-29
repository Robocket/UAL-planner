#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
NOETIC_SETUP="${ROS_NOETIC_SETUP:-/opt/ros/noetic/setup.bash}"

if [[ ! -r "${NOETIC_SETUP}" ]]; then
    echo "找不到 ROS Noetic 环境: ${NOETIC_SETUP}" >&2
    exit 1
fi

exec env \
    -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u LD_LIBRARY_PATH \
    -u PKG_CONFIG_PATH \
    -u PYTHONPATH \
    -u ROS_DISTRO \
    -u ROS_VERSION \
    -u ROS_PYTHON_VERSION \
    -u RMW_IMPLEMENTATION \
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    bash --noprofile --norc -c '
        set -eo pipefail
        source "$1"
        set -u
        cd "$2"
        echo "Building with ROS_DISTRO=${ROS_DISTRO}"
        catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 "${@:3}"
    ' bash "${NOETIC_SETUP}" "${WORKSPACE_ROOT}" "$@"
