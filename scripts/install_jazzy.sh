#!/usr/bin/env bash
# Official ROS apt-source setup plus the packages required by this repository.
# Run in a local terminal: sudo bash scripts/install_jazzy.sh
set -euo pipefail

if (( EUID != 0 )); then
    printf '需要管理员权限，请在本机终端执行：\n  sudo bash %q\n' "${BASH_SOURCE[0]}" >&2
    exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_CODENAME:-}" != noble ]]; then
    echo '此脚本仅支持 Ubuntu 24.04 Noble / ROS 2 Jazzy。' >&2
    exit 1
fi
if [[ "$(dpkg --print-architecture)" != amd64 && "$(dpkg --print-architecture)" != arm64 ]]; then
    echo '此脚本仅支持官方 Noble amd64/arm64 二进制包。' >&2
    exit 1
fi
root_free_kb="$(df -Pk / | awk 'NR == 2 {print $4}')"
if (( root_free_kb < 2 * 1024 * 1024 )); then
    echo '根分区可用空间小于 2 GiB，请先腾出空间；脚本不会自动删除文件。' >&2
    exit 1
fi
df -h /
export LANG=C.UTF-8 LC_ALL=C.UTF-8

apt-get update --error-on=any
apt-get install --no-remove --no-install-recommends -y \
    ca-certificates curl software-properties-common
add-apt-repository -y universe

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -qx 'install ok installed'; then
    ros_apt_version="$(curl --fail --silent --show-error --location \
        --retry 3 --connect-timeout 20 --max-time 120 \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')"
    if [[ ! "$ros_apt_version" =~ ^[0-9][0-9A-Za-z.+~-]*$ ]]; then
        echo 'ROS apt-source release version is invalid.' >&2
        exit 1
    fi
    ros_apt_dir="$(mktemp -d /tmp/ual-jazzy-apt.XXXXXX)"
    ros_apt_deb="${ros_apt_dir}/ros2-apt-source.deb"
    curl --fail --show-error --location --retry 3 --connect-timeout 20 --max-time 180 \
        --output "$ros_apt_deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.noble_all.deb"
    if [[ "$(dpkg-deb -f "$ros_apt_deb" Package)" != ros2-apt-source ]]; then
        echo 'Downloaded package is not ros2-apt-source.' >&2
        exit 1
    fi
    dpkg -i "$ros_apt_deb"
fi

apt-get update --error-on=any
# ros-base + RViz keeps the install smaller than the complete desktop bundle.
# --no-remove prevents dependency resolution from removing existing software.
apt-get install --no-remove --no-install-recommends -y \
    ros-jazzy-ros-base ros-jazzy-rviz2 ros-dev-tools \
    ros-jazzy-ament-cmake-python ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-pytest ros-jazzy-cv-bridge \
    ros-jazzy-diagnostic-msgs ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs-py ros-jazzy-visualization-msgs \
    ros-jazzy-tf2-ros ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime ros-jazzy-launch-ros \
    build-essential cmake git libopencv-dev libcurl4-openssl-dev libssl-dev \
    python3-dev python3-numpy python3-opencv python3-yaml

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
fi

env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH -u LD_LIBRARY_PATH \
    bash --noprofile --norc -c '
        set -e
        source /opt/ros/jazzy/setup.bash
        test "$ROS_DISTRO" = jazzy
        ros2 pkg prefix rviz2
        colcon --help >/dev/null
        python3 -c "import rclpy, cv_bridge, sensor_msgs_py; print(\"ROS 2 Jazzy Python dependencies: OK\")"
    '

printf '\nROS 2 Jazzy 和 UAL 系统依赖安装完成。未修改 .bashrc，未下载分割模型。\n'
printf '在普通用户终端运行：\n  source /opt/ros/jazzy/setup.bash\n  rosdep update --rosdistro jazzy\n'
printf '然后在仓库运行：\n  ./scripts/build_jazzy.sh\n'
df -h /
