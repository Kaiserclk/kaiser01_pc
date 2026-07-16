import rclpy
from rclpy.node import Node
import time
from typing import Optional, Callable, Any
from dataclasses import dataclass


@dataclass
class TestResult:
    """Test result data class"""
    success: bool
    topic_name: str
    time_diff: Optional[float] = None
    message: str = ""
    data_info: dict = None

    def __post_init__(self):
        if self.data_info is None:
            self.data_info = {}


class TopicTester(Node):
    """Base class for topic testing"""

    def __init__(self, topic_name: str, msg_type: Any, timeout_sec: float = 5.0):
        super().__init__(f'{topic_name.replace("/", "_").strip("_")}_tester')
        self.topic_name = topic_name
        self.timeout_sec = timeout_sec
        self.received_data = False
        self.last_msg = None
        self.time_diff = None
        
        # Subscribe to topic
        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self._callback,
            10
        )
        self.get_logger().info(f'Waiting for {topic_name} topic data...')

    def _callback(self, msg):
        """Message callback"""
        self.received_data = True
        self.last_msg = msg
        
        # Calculate time difference
        if hasattr(msg, 'header'):
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            current_time = self.get_clock().now()
            current_time_sec = current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1] * 1e-9
            self.time_diff = current_time_sec - msg_time
        
        # Call subclass message processing method
        self.process_message(msg)

    def process_message(self, msg):
        """Override this method in subclass to process specific messages"""
        pass

    def get_data_info(self) -> dict:
        """Override this method in subclass to return specific data info"""
        return {}

    def test(self) -> TestResult:
        """Execute test"""
        start_time = time.time()
        
        # Wait for data or timeout
        while not self.received_data and (time.time() - start_time) < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.received_data:
            return TestResult(
                success=False,
                topic_name=self.topic_name,
                message=f"Timeout: no data received within {self.timeout_sec} seconds"
            )
        
        # Build result
        data_info = self.get_data_info()
        msg = f"Data OK"
        
        if self.time_diff is not None:
            if abs(self.time_diff) > 1.0:
                msg = f"Timestamp sync abnormal, diff: {self.time_diff:.4f}s"
            else:
                msg = f"Timestamp normal, diff: {self.time_diff:.4f}s"
        
        return TestResult(
            success=True,
            topic_name=self.topic_name,
            time_diff=self.time_diff,
            message=msg,
            data_info=data_info
        )


class ScanTester(TopicTester):
    """Test /scan topic"""

    def __init__(self, timeout_sec: float = 5.0):
        from sensor_msgs.msg import LaserScan
        super().__init__('/scan', LaserScan, timeout_sec)
        self.data_info = {}

    def process_message(self, msg):
        ranges = [r for r in msg.ranges if r < msg.range_max]
        self.data_info = {
            'data_points': len(msg.ranges),
            'valid_points': len(ranges),
            'angle_range': f"{msg.angle_min:.2f} ~ {msg.angle_max:.2f} rad",
            'min_distance': min(ranges) if ranges else None
        }

    def get_data_info(self) -> dict:
        return self.data_info


class OdomTester(TopicTester):
    """Test /odom topic"""

    def __init__(self, timeout_sec: float = 5.0):
        from nav_msgs.msg import Odometry
        super().__init__('/odom', Odometry, timeout_sec)
        self.data_info = {}

    def process_message(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.data_info = {
            'position': f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})",
            'orientation': f"({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f})",
            'linear_velocity': f"({twist.linear.x:.3f}, {twist.linear.y:.3f}, {twist.linear.z:.3f})",
            'angular_velocity': f"({twist.angular.x:.3f}, {twist.angular.y:.3f}, {twist.angular.z:.3f})"
        }

    def get_data_info(self) -> dict:
        return self.data_info


class ImuTester(TopicTester):
    """Test /imu/data_raw topic"""

    def __init__(self, timeout_sec: float = 5.0):
        from sensor_msgs.msg import Imu
        super().__init__('/imu/data_raw', Imu, timeout_sec)
        self.data_info = {}

    def process_message(self, msg):
        self.data_info = {
            'angular_velocity': f"({msg.angular_velocity.x:.4f}, {msg.angular_velocity.y:.4f}, {msg.angular_velocity.z:.4f})",
            'linear_acceleration': f"({msg.linear_acceleration.x:.4f}, {msg.linear_acceleration.y:.4f}, {msg.linear_acceleration.z:.4f})",
            'orientation': f"({msg.orientation.x:.4f}, {msg.orientation.y:.4f}, {msg.orientation.z:.4f}, {msg.orientation.w:.4f})"
        }

    def get_data_info(self) -> dict:
        return self.data_info


# Topic tester mapping
TOPIC_TESTERS = {
    'scan': ScanTester,
    'odom': OdomTester,
    'imu': ImuTester
}


def print_result(result: TestResult):
    """Print test result"""
    if result.success:
        print(f"✅ {result.topic_name}: {result.message}")
        if result.data_info:
            for key, value in result.data_info.items():
                print(f"   {key}: {value}")
    else:
        print(f"❌ {result.topic_name}: {result.message}")


def test_topic(topic_name: str, timeout_sec: float = 5.0) -> TestResult:
    """
    Test specified topic
    
    Args:
        topic_name: Topic name (scan, odom, imu)
        timeout_sec: Timeout for waiting data (seconds)
    
    Returns:
        TestResult: Test result
    """
    if topic_name not in TOPIC_TESTERS:
        return TestResult(
            success=False,
            topic_name=topic_name,
            message=f"Unknown topic: {topic_name}, supported: {list(TOPIC_TESTERS.keys())}"
        )
    
    rclpy.init()
    try:
        tester = TOPIC_TESTERS[topic_name](timeout_sec=timeout_sec)
        result = tester.test()
        print_result(result)
        return result
    finally:
        tester.destroy_node()
        rclpy.shutdown()


def test_scan(timeout_sec: float = 5.0) -> TestResult:
    """Test /scan topic"""
    return test_topic('scan', timeout_sec)


def test_odom(timeout_sec: float = 5.0) -> TestResult:
    """Test /odom topic"""
    return test_topic('odom', timeout_sec)


def test_imu(timeout_sec: float = 5.0) -> TestResult:
    """Test /imu/data_raw topic"""
    return test_topic('imu', timeout_sec)


def test_all(timeout_sec: float = 5.0) -> dict:
    """
    Test all supported topics
    
    Returns:
        dict: Topic name -> TestResult
    """
    rclpy.init()
    results = {}
    try:
        # Create all testers
        testers = {name: cls(timeout_sec=timeout_sec) for name, cls in TOPIC_TESTERS.items()}
        
        # Wait for all tests to complete
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(list(testers.values())[0], timeout_sec=0.1)
            
            # Check if all topics received data
            all_received = all(t.received_data for t in testers.values())
            if all_received:
                break
        
        # Get results
        for name, tester in testers.items():
            results[name] = tester.test()
            print_result(results[name])
        
        return results
    finally:
        for tester in testers.values():
            tester.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """CLI entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='ROS2 Topic Test Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run robot_util robot_test --topic scan
  ros2 run robot_util robot_test --topic odom --timeout 10
  ros2 run robot_util robot_test --topic all
        """
    )
    parser.add_argument('--topic', '-t', type=str, default='all',
                       choices=list(TOPIC_TESTERS.keys()) + ['all'],
                       help='Topic to test (default: all)')
    parser.add_argument('--timeout', type=float, default=5.0,
                       help='Timeout in seconds (default: 5.0)')
    
    parsed_args = parser.parse_args(args)
    
    if parsed_args.topic == 'all':
        test_all(timeout_sec=parsed_args.timeout)
    else:
        test_topic(parsed_args.topic, timeout_sec=parsed_args.timeout)


if __name__ == '__main__':
    main()
