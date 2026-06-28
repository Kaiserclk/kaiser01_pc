#!/bin/bash
# 手眼标定便捷采样脚本
# 使用方法: ./sample.sh

echo "==========================================="
echo "手眼标定 - 数据采集"
echo "==========================================="
echo ""
echo "提示:"
echo "1. 确保机械臂已移动到目标位置"
echo "2. 确保机械臂已稳定(等待2-3秒)"
echo "3. 确保相机能清晰看到标定板"
echo ""
read -p "按 Enter 键开始采样, 或按 Ctrl+C 取消..."

echo ""
echo "正在调用采样服务..."
echo ""

# 调用 ROS2 服务
ros2 service call /handeye_calibration/sample std_srvs/srv/Trigger

echo ""
echo "==========================================="
