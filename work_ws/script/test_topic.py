#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointTrajectoryTester(Node):
    def __init__(self):
        super().__init__('joint_trajectory_tester')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        # 等 1 秒确保连接建立
        self.create_timer(1.0, self.send_trajectory)

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3',
            'arm_joint4', 'arm_joint5'
        ]

        # 添加多个轨迹点，实现平滑运动
        waypoints = [
            # ([0.0, 0.0, 0.0, 0.0, 0.0], 2.0),
            ([0.3, -0.6, 1.2, 1.3, 0.15], 1.0),   # 按需添加更多点
        ]

        for positions, seconds in waypoints:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=int(seconds), nanosec=0)
            msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info('Trajectory sent!')
        self.destroy_node()


def main():
    rclpy.init()
    node = JointTrajectoryTester()
    rclpy.spin_once(node, timeout_sec=3)


if __name__ == '__main__':
    main()