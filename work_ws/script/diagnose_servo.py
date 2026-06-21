#!/usr/bin/env python3
"""诊断 servo_node 数据流问题"""
import sys
import subprocess
import time

def run_cmd(cmd, timeout=3):
    """运行命令并返回输出"""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.stdout.strip()
    except:
        return ""

def check_topic_rate(topic, duration=3):
    """检查话题发布频率"""
    cmd = f"ros2 topic hz {topic} -w {duration} 2>&1 | tail -1"
    return run_cmd(cmd)

def main():
    print("="*60)
    print("servo_node 数据流诊断")
    print("="*60)
    
    # 1. 检查 servo_node 状态
    print("\n1️⃣ 检查 servo_node 状态:")
    status = run_cmd("ros2 topic echo /servo_node/status --once 2>/dev/null | grep 'data:'")
    print(f"   {status if status else '❌ 没有状态数据'}")
    
    # 2. 检查 /joint_states 是否有数据
    print("\n2️⃣ 检查 /joint_states:")
    joint_states = run_cmd("ros2 topic echo /joint_states --once 2>/dev/null | head -3")
    if joint_states:
        print("   ✅ /joint_states 有数据")
        print(f"   {joint_states}")
    else:
        print("   ❌ /joint_states 没有数据！servo_node 需要关节状态才能计算 IK")
    
    # 3. 检查 /servo_node/pose_target_cmds
    print("\n3️⃣ 检查 /servo_node/pose_target_cmds:")
    pose_cmd = run_cmd("ros2 topic echo /servo_node/pose_target_cmds --once 2>/dev/null | head -5")
    if pose_cmd:
        print("   ✅ pose_target_cmds 有数据")
    else:
        print("   ❌ pose_target_cmds 没有数据")
    
    # 4. 检查 /arm_controller/joint_trajectory
    print("\n4️⃣ 检查 /arm_controller/joint_trajectory:")
    jt = run_cmd("ros2 topic echo /arm_controller/joint_trajectory --once 2>/dev/null | head -3")
    if jt:
        print("   ✅ joint_trajectory 有数据")
    else:
        print("   ❌ joint_trajectory 没有数据！")
    
    # 5. 检查话题频率
    print("\n5️⃣ 检查话题发布频率（3秒）:")
    print(f"   pose_target_cmds: {check_topic_rate('/servo_node/pose_target_cmds')}")
    print(f"   joint_trajectory: {check_topic_rate('/arm_controller/joint_trajectory')}")
    
    # 6. 检查服务
    print("\n6️⃣ 检查 servo_node 服务:")
    services = run_cmd("ros2 service list | grep servo")
    print(f"   {services}")
    
    # 7. 检查 servo_node 进程
    print("\n7️⃣ 检查 servo_node 进程:")
    ps = run_cmd("ps aux | grep servo_node | grep -v grep")
    if ps:
        print("   ✅ servo_node 进程运行中")
    else:
        print("   ❌ servo_node 进程未运行")
    
    print("\n" + "="*60)
    print("诊断建议:")
    print("="*60)
    
    if "data: 0" in status:
        print("⚠️  servo 处于 STOPPED 状态，需要调用 start_servo 服务激活")
    elif "data: 1" in status:
        print("⚠️  servo 处于 LOW_GAIN 状态，可能需要调整参数")
    elif "data: 2" in status:
        print("✅ servo 已激活（ACTIVE）")
    else:
        print("⚠️  无法获取 servo 状态")
    
    if not joint_states:
        print("❌ 缺少 /joint_states 话题！")
        print("   解决方案: 启动 joint_state_publisher 或确保 ros2_control 硬件接口发布关节状态")
        print("   ros2 run joint_state_publisher joint_state_publisher")
    
    print("\n完整启动命令（确保所有组件运行）:")
    print("  终端1: ros2 launch kai01_bringup realtime_servo.launch.py")
    print("  终端2: python3 keyboard_servo.py")

if __name__ == "__main__":
    main()
