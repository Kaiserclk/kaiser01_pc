#!/usr/bin/env python3
"""简化版 servo 测试 - 直接发送固定位姿"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import Int8
import time

class SimpleServoTest(Node):
    def __init__(self):
        super().__init__("simple_servo_test")
        
        # 订阅状态
        self.status_sub = self.create_subscription(
            Int8, "/servo_node/status", self.status_callback, 10
        )
        self.last_status = -1
        
        # 发布位姿命令
        self.pose_pub = self.create_publisher(
            PoseStamped, "/servo_node/pose_target_cmds", 10
        )
        
        # 调用 start_servo
        self.start_cli = self.create_client(Trigger, "/servo_node/start_servo")
        
    def status_callback(self, msg):
        status_names = {0: "STOPPED", 1: "STARTING", 2: "ACTIVE"}
        if msg.data != self.last_status:
            self.get_logger().info(f"状态: {status_names.get(msg.data, f'UNKNOWN({msg.data})')}")
            self.last_status = msg.data
    
    def activate_servo(self):
        self.get_logger().info("调用 start_servo...")
        if self.start_cli.wait_for_service(timeout_sec=5.0):
            future = self.start_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result() and future.result().success:
                self.get_logger().info("✅ servo 已激活")
                return True
        self.get_logger().error("❌ 激活失败")
        return False
    
    def send_test_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose)
        self.get_logger().info(f"发送位姿: x={x}, y={y}, z={z}")

def main():
    rclpy.init()
    node = SimpleServoTest()
    
    # 激活 servo
    if not node.activate_servo():
        return
    
    # 等待状态更新
    time.sleep(1)
    
    # 持续发送位姿命令
    try:
        count = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # 每 0.1 秒发送一次
            if count % 10 == 0:
                # 在小范围内移动
                x = 0.3 + 0.02 * (count % 10) / 10.0
                node.send_test_pose(x, 0.0, 0.3)
            
            count += 1
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
