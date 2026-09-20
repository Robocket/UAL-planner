#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"

SEYOND_WS="${HOME}/ws_crl/Seyond_Driver"
LANDING_WS="${HOME}/ws_crl/Landing_Obstacle_Warning"

SEYOND_SETUP="${SEYOND_WS}/install/setup.bash"
LANDING_SETUP="${LANDING_WS}/install/setup.bash"

BAG_ROOT="${HOME}/landing_function_test_bags"
BAG_NAME="D1_function_test_$(date +%Y%m%d_%H%M%S)"
SESSION="d1_test"

mkdir -p "${BAG_ROOT}"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "已有 ${SESSION}，请先执行:"
    echo "tmux kill-session -t ${SESSION}"
    exit 1
fi

# ============================================================
# 1. 录包节点先加载所有 ROS 环境
#    避免 rosbag2 无法识别自定义 msg 类型
# ============================================================

tmux new-session -d -s "${SESSION}" -n bag

tmux send-keys -t "${SESSION}:bag" \
"source ${ROS_SETUP}; source ${SEYOND_SETUP}; source ${LANDING_SETUP}; ros2 bag record -a -o ${BAG_ROOT}/${BAG_NAME}" C-m

echo "[1/3] 全量录包启动（已加载全部工作空间）"

sleep 5

# ============================================================
# 2. 启动 Seyond 驱动
# ============================================================

tmux new-window -t "${SESSION}" -n radar

tmux send-keys -t "${SESSION}:radar" \
"source ${ROS_SETUP}; source ${SEYOND_SETUP}; cd ${SEYOND_WS}; ros2 launch seyond start.py" C-m

echo "[2/3] Seyond 驱动启动"

sleep 5

# ============================================================
# 3. 启动 Landing_Obstacle_Warning
# ============================================================

tmux new-window -t "${SESSION}" -n landing

tmux send-keys -t "${SESSION}:landing" \
"source ${ROS_SETUP}; source ${SEYOND_SETUP}; source ${LANDING_SETUP}; cd ${LANDING_WS}; ros2 launch landing_evaluator d1_test.launch.py" C-m

echo "[3/3] Landing_Obstacle_Warning 启动"

echo
echo "录包目录:"
echo "${BAG_ROOT}/${BAG_NAME}"
echo
echo "查看:"
echo "tmux attach -t ${SESSION}"
echo
echo "结束:"
echo "在 bag 窗口 Ctrl+C 停止录包"
echo "tmux kill-session -t ${SESSION}"