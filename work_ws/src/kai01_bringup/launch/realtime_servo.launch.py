import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    """Load a YAML file from a package's share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Build MoveIt configuration (includes robot_description, robot_description_semantic,
    # robot_description_kinematics, joint_limits, etc.)
    moveit_config = (
        MoveItConfigsBuilder("kai01_robot", package_name="moveit_config")
        .to_moveit_configs()
    )

    # Load servo parameters from moveit_config package
    servo_yaml = load_yaml("moveit_config", "config/servo_params.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    
    # Humble: 添加启动参数让 servo_node 自动激活
    servo_params["moveit_servo"].update({
        "incoming_command_timeout": 0.1,
    })

    # Launch Servo as a standalone node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,  # IK plugin parameters from kinematics.yaml
        ],
        output="screen",
    )

    # 创建一个节点，在 servo_node 启动后自动调用 start_servo 服务
    activate_servo = Node(
        package="ros2pkg",
        executable="ros2pkg",
        name="activate_servo",
        output="screen",
        on_exit=[
            # 使用 TimerAction 延迟调用 start_servo
            TimerAction(
                period=2.0,  # 等待 2 秒让 servo_node 完全启动
                actions=[
                    # 通过 ros2 service call 调用 start_servo
                    # 注意：这需要在系统中有 ros2 命令可用
                ]
            )
        ]
    )
    
    # 更简单的方法：使用 shell 命令在延迟后调用服务
    from launch.actions import ExecuteProcess
    activate_servo_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen',
        shell=False
    )
    
    # 延迟 2 秒后执行
    delayed_activate = TimerAction(
        period=2.0,
        actions=[activate_servo_cmd]
    )

    return LaunchDescription([servo_node, delayed_activate])
