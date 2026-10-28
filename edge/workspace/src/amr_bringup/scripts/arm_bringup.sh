#!/bin/bash
# 止める前に腕をstandbyへ動かす(launchを直接止めるとトルクが抜けて腕が落ちるため)。
# set -mが無いとバックグラウンドのlaunchがSIGINTを無視し、トルクを切らずに強制終了される。
set -m
ros2 launch amr_bringup arm.launch.xml "$@" &
launch_pid=$!

stop() {
    trap - TERM INT
    ros2 run amr_bringup move_arm_to_standby.py --ros-args --params-file "$(ros2 pkg prefix amr_bringup)/share/amr_bringup/config/arm_standby.yaml"
    kill -INT "$launch_pid"
}
trap stop TERM INT

# waitはtrapで中断されるため、launchが終わるまで待ち直す。
while kill -0 "$launch_pid" 2>/dev/null; do
    wait "$launch_pid"
done
