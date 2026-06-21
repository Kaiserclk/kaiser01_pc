#!/usr/bin/env python3
"""测试 servo 末端位姿控制 - 验证完整数据流"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from tf_transformations import quaternion_from_euler
import time


class ServoPoseTest(Node):
    def __init__(self):
        super().__init__("servo_pose_test")
        
        # 订阅状态
        self.status_sub = self.create_subscription(
            Int8,
            "/servo_node/status",
            self.status_callback,
            10
        )
        self.servo_active = False
        
        # 发布位姿命令
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/servo_node/pose_target_cmds",
            10
        )
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Servo 末端位姿控制测试")
        self.get_logger().info("=" * 60)

    def status_callback(self, msg):
        status_map = {0: "STOPPED", 1: "STARTING", 2: "ACTIVE"}
        status_name = status_map.get(msg.data, f"UNKNOWN({msg.data})")
        
        if msg.data == 2 and not self.servo_active:
            self.get_logger().info("✅ servo 已激活！")
            self.servo_active = True
        elif msg.data != 2:
            self.get_logger().info(f"⏳ servo 状态: {status_name}")

    def send_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """发送目标位姿"""
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.pose_pub.publish(msg)
        self.get_logger().info(
            f"📤 发送位姿: x={x:.3f} y={y:.3f} z={z:.3f} | "
            f"R={roll:.2f} P={pitch:.2f} Y={yaw:.2f}"
        )


def main():
    rclpy.init()
    node = ServoPoseTest()
    
    # 等待 3 秒让话题连接
    time.sleep(3)
    
    if not node.servo_active:
        node.get_logger().warn("⚠️ servo 未激活，但仍然尝试发送命令...")
    
    # 测试 1: 初始位姿
    node.send_pose(0.3, 0.0, 0.3, 0.0, 0.0, 0.0)
    time.sleep(2)
    
    # 测试 2: 移动 X
    node.send_pose(0.35, 0.0, 0.3, 0.0, 0.0, 0.0)
    time.sleep(2)
    
    # 测试 3: 移动 Z
    node.send_pose(0.35, 0.0, 0.35, 0.0, 0.0, 0.0)
    time.sleep(2)
    
    # 测试 4: 旋转
    node.send_pose(0.35, 0.0, 0.35, 0.1, 0.0, 0.0)
    time.sleep(2)
    
    node.get_logger().info("=" * 60)
    node.get_logger().info("测试完成！")
    node.get_logger().info("请检查:")
    node.get_logger().info("  1. /servo_node/status 是否变为 2 (ACTIVE)")
    node.get_logger().info("  2. /arm_controller/joint_trajectory 是否有数据")
    node.get_logger().info("  3. 机械臂是否移动")
    node.get_logger().info("=" * 60)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
