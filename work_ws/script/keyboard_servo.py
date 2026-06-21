#!/usr/bin/env python3
import sys
import tty
import termios
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Trigger
from std_msgs.msg import Int8


BINDINGS = {
    "w": ("x", 0.02),
    "s": ("x", -0.02),
    "a": ("y", 0.02),
    "d": ("y", -0.02),
    "q": ("z", 0.02),
    "e": ("z", -0.02),
    "j": ("rx", 0.05),
    "l": ("rx", -0.05),
    "i": ("ry", 0.05),
    "k": ("ry", -0.05),
    "u": ("rz", 0.05),
    "o": ("rz", -0.05),
}


class KeyboardServo(Node):
    def __init__(self):
        super().__init__("keyboard_servo")
        
        # 发布位姿命令到 servo_node
        self.pub = self.create_publisher(PoseStamped, "/servo_node/pose_target_cmds", 10)
        
        # 订阅 servo 状态
        self.status_sub = self.create_subscription(
            Int8,
            "/servo_node/status",
            self.status_callback,
            10
        )
        self.servo_active = False

        # 初始化目标位姿（相对于 base_link）
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "base_link"
        self.target_pose.pose.position.x = 0.3
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.3
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        self.target_pose.pose.orientation.x = q[0]
        self.target_pose.pose.orientation.y = q[1]
        self.target_pose.pose.orientation.z = q[2]
        self.target_pose.pose.orientation.w = q[3]

        self.get_logger().info("=" * 60)
        self.get_logger().info("键盘伺服控制 - 末端位姿模式（位置控制）")
        self.get_logger().info("=" * 60)
        self.get_logger().info("初始位姿: x=0.3, y=0, z=0.3, rpy=0")
        self.get_logger().info("按键: wasd=前后左右 qe=升降 jlikuo=旋转 空格=重置")
        self.get_logger().info("=" * 60)

    def status_callback(self, msg):
        """监听 servo 状态"""
        status_map = {0: "STOPPED", 1: "STARTING", 2: "ACTIVE"}
        status_name = status_map.get(msg.data, f"UNKNOWN({msg.data})")
        
        if msg.data == 2:  # ACTIVE
            if not self.servo_active:
                self.get_logger().info("✅ servo 已激活，可以开始控制！")
            self.servo_active = True
        else:
            if self.servo_active:
                self.get_logger().warn(f"⚠️ servo 状态: {status_name}")
            self.servo_active = (msg.data == 2)

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def update_and_publish(self, key):
        """更新位姿并发布"""
        if key == " ":
            # 重置到初始位姿
            self.target_pose.pose.position.x = 0.3
            self.target_pose.pose.position.y = 0.0
            self.target_pose.pose.position.z = 0.3
            q = quaternion_from_euler(0.0, 0.0, 0.0)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.get_logger().info("↺ 位姿已重置")
            return

        if key not in BINDINGS:
            return

        axis, delta = BINDINGS[key]

        # 更新位置
        if axis == "x":
            self.target_pose.pose.position.x += delta
        elif axis == "y":
            self.target_pose.pose.position.y += delta
        elif axis == "z":
            self.target_pose.pose.position.z += delta
        # 更新姿态（欧拉角转四元数）
        elif axis in ("rx", "ry", "rz"):
            rpy = euler_from_quaternion([
                self.target_pose.pose.orientation.x,
                self.target_pose.pose.orientation.y,
                self.target_pose.pose.orientation.z,
                self.target_pose.pose.orientation.w,
            ])
            if axis == "rx":
                rpy[0] += delta
            elif axis == "ry":
                rpy[1] += delta
            elif axis == "rz":
                rpy[2] += delta
            q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]

        # 打印当前目标位姿
        pos = self.target_pose.pose.position
        rpy = euler_from_quaternion([
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w,
        ])
        self.get_logger().info(
            f"目标: x={pos.x:.3f} y={pos.y:.3f} z={pos.z:.3f} | "
            f"R={math.degrees(rpy[0]):.1f}° P={math.degrees(rpy[1]):.1f}° Y={math.degrees(rpy[2]):.1f}°"
        )

        # 发布位姿命令
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.target_pose)


def main():
    rclpy.init()
    node = KeyboardServo()

    def spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

    # 先启动 spin 线程，再调用服务（避免 spin_until_future_complete 死锁）
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    # Humble: servo 启动后默认是 STOPPED，必须主动调用 start_servo 激活
    node.get_logger().info("🔧 正在调用 /servo_node/start_servo 服务...")
    start_cli = node.create_client(Trigger, "/servo_node/start_servo")
    
    if start_cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().info("✅ start_servo 服务已找到，正在调用...")
        future = start_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        
        if future.done():
            result = future.result()
            if result is not None:
                if result.success:
                    node.get_logger().info("✅ start_servo 调用成功，servo 已激活")
                else:
                    node.get_logger().error(f"❌ start_servo 调用失败: {result.message}")
            else:
                node.get_logger().error("❌ start_servo 返回结果为 None")
        else:
            node.get_logger().error("❌ start_servo 调用超时")
            # 尝试再次调用
            node.get_logger().info("🔄 尝试第二次调用 start_servo...")
            future = start_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            if future.done() and future.result() and future.result().success:
                node.get_logger().info("✅ 第二次调用 start_servo 成功")
            else:
                node.get_logger().error("❌ 第二次调用 start_servo 也失败了")
    else:
        node.get_logger().error("❌ /servo_node/start_servo 服务不存在，请确认 servo_node 已启动")

    try:
        while rclpy.ok():
            key = node.get_key()
            if key == "\x03":
                break
            if key in BINDINGS or key == " ":
                node.update_and_publish(key)
            else:
                continue
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()