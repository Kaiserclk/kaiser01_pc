#!/bin/bash
# 手眼标定执行脚本
# 使用方法: ./calibrate.sh

echo "==========================================="
echo "手眼标定 - 执行计算"
echo "==========================================="
echo ""

# 调用标定服务
ros2 service call /handeye_calibration/calibrate std_srvs/srv/Trigger

echo ""
echo "==========================================="
echo ""
echo "标定结果已保存到: /tmp/handeye_calibration_result.yaml"
echo ""
echo "查看结果:"
echo "  cat /tmp/handeye_calibration_result.yaml"
echo ""
echo "==========================================="
