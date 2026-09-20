#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
JAZZY_SETUP="${ROS_JAZZY_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -r "${JAZZY_SETUP}" ]]; then
    echo "找不到 ROS 2 Jazzy 环境: ${JAZZY_SETUP}" >&2
    exit 1
fi

# Start a non-interactive shell without inherited ROS/CMake/Python prefixes.
# This prevents an earlier Noetic/Humble/Jazzy source command from creating a
# mixed CMake cache. HOME and other user settings are inherited unchanged.
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
        colcon --log-base log/jazzy build \
            --build-base build/jazzy \
            --install-base install/jazzy \
            --symlink-install \
            "${@:3}"
    ' bash "${JAZZY_SETUP}" "${WORKSPACE_ROOT}" "$@"
