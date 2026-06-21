import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_configuration_directory_cmd = DeclareLaunchArgument(
        'configuration_directory',
        default_value=os.path.join(get_package_share_directory('kai01_bringup'),
                                   'config'),
        description='Full path to the cartographer configuration directory')

    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='kai01_2d.lua',
        description='Name of the cartographer configuration file')

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    # Occupancy grid node - converts cartographer submaps to a 2D occupancy grid
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ]
    )

    # RViz2
    rviz_config_file = os.path.join(get_package_share_directory('kai01_bringup'),
                                    'rviz', 'cartographer_default.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_configuration_directory_cmd)
    ld.add_action(declare_configuration_basename_cmd)

    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
