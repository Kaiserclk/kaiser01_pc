#!/usr/bin/env python3
"""
Hand-Eye Calibration Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取功能包路径
    pkg_share = get_package_share_directory('handeye_calibration')
    
    # 参数配置
    parameters = [
        {'base_frame': 'base_link'},
        {'end_effector_frame': 'gripper_tcp'},
        {'camera_frame': 'camera_color_optical_frame'},
        {'target_type': 'Charuco'},  # Aruco 或 Charuco
        {'mount_type': 'eye_in_hand'},  # eye_in_hand 或 eye_to_hand
        {'min_samples': 10},
        {'solver_method': 'Tsai1989'},
        {'output_file': '/tmp/handeye_calibration_result.yaml'},
    ]
    
    # 手眼标定节点
    handeye_calibrator_node = Node(
        package='handeye_calibration',
        executable='handeye_calibrator',
        name='handeye_calibrator',
        output='screen',
        parameters=parameters,

    )
    
    return LaunchDescription([
        handeye_calibrator_node,
    ])
