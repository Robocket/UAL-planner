#!/usr/bin/env bash
# One command to load Jazzy, replay the converted bag and display camera/LiDAR.
set -euo pipefail

PLAY_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PLAY_REPO_ROOT="$(cd -- "${PLAY_SCRIPT_DIR}/.." && pwd)"
PLAY_JAZZY_SETUP="${ROS_JAZZY_SETUP:-/opt/ros/jazzy/setup.bash}"
PLAY_LAUNCH="${PLAY_REPO_ROOT}/launch/play_scene_water.launch.py"

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    printf '%s\n' \
        '用法：play_scene_water.sh [ROS 2 launch 参数]' \
        '默认循环播放，并打开 RViz 显示图像和点云；Ctrl+C 退出。' \
        '' \
        '示例：' \
        '  ./play.sh                             # 循环播放并显示' \
        '  ./play.sh loop:=false                 # 播放一次' \
        '  ./play.sh rate:=0.5                   # 半速播放' \
        '  ./play.sh start_offset:=45            # 从第 45 秒开始' \
        '  ./play.sh start_rviz:=false           # 只回放，不开 RViz' \
        '  ./play.sh bag:=/absolute/path/to/bag   # 指定其他 ROS 2 bag'
    exit 0
fi
if [[ ! -r "$PLAY_JAZZY_SETUP" ]]; then
    printf '找不到 ROS 2 Jazzy 环境：%s\n' "$PLAY_JAZZY_SETUP" >&2
    exit 1
fi
if [[ ! -r "$PLAY_LAUNCH" ]]; then
    printf '找不到回放 launch：%s\n' "$PLAY_LAUNCH" >&2
    exit 1
fi

# Load only Jazzy, independent of the calling terminal's ROS distribution.
# exec keeps Ctrl+C connected to launch, its player and RViz.
exec env \
    -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    bash --noprofile --norc -c '
        set -eo pipefail
        source "$1"
        exec ros2 launch "$2" "${@:3}"
    ' bash "$PLAY_JAZZY_SETUP" "$PLAY_LAUNCH" "$@"
