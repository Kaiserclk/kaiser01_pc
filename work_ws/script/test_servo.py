#!/usr/bin/env python3
"""诊断 servo 状态并测试控制"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
import time


class ServoDiagnostics(Node):
    def __init__(self):
        super().__init__("servo_diagnostics")

        # 检查 start_servo 服务
        self.start_cli = self.create_client(Trigger, "/servo_node/start_servo")
        
        # 订阅状态话题
        self.status_sub = self.create_subscription(
            std_msgs.msg.Int8,
            "/servo_node/status",
            self.status_callback,
            10
        )
        self.last_status = -1

        # 发布测试命令
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10
        )

    def status_callback(self, msg):
        if msg.data != self.last_status:
            status_map = {0: "STOPPED", 1: "STARTING", 2: "ACTIVE"}
            status_name = status_map.get(msg.data, f"UNKNOWN({msg.data})")
            self.get_logger().info(f"Servo 状态: {status_name} ({msg.data})")
            self.last_status = msg.data

    def start_servo(self):
        self.get_logger().info("尝试启动 servo...")
        if not self.start_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("start_servo 服务不存在！")
            return False
        
        req = Trigger.Request()
        future = self.start_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("✅ servo 启动成功！")
            return True
        else:
            self.get_logger().error(f"❌ servo 启动失败: {future.result().message}")
            return False

    def send_test_command(self):
        """发送一个简单的测试命令"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.05  # 小幅度前进
        self.cmd_pub.publish(msg)
        self.get_logger().info("发送测试命令: linear.x=0.05")


def main():
    rclpy.init()
    node = ServoDiagnostics()

    # 等待几秒让话题连接
    time.sleep(2)

    # 启动 servo
    if node.start_servo():
        # 等待状态更新
        time.sleep(1)
        
        # 发送测试命令
        node.send_test_command()
        node.get_logger().info("请在另一个终端检查 /arm_controller/joint_trajectory 是否有数据")
        node.get_logger().info("等待 3 秒后退出...")
        time.sleep(3)
    else:
        node.get_logger().error("servo 无法启动，请检查配置")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
