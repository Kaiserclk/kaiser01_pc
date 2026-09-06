import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    map_folder_dir = os.path.join(os.path.expanduser('~'), 'work_ws' ,'map')

    declared_arguments = [
        DeclareLaunchArgument(
            'map_name',
            default_value='map',
            description='Map name to save, without file extension',
        ),
        DeclareLaunchArgument(
            'map_type',
            default_value='gridmap',
            description='Type of map to be saved (optional):gridmap、pbstream、posegraph',
        ),
    ]

    map_name = LaunchConfiguration('map_name')
    map_type = LaunchConfiguration('map_type')
    map_path = PathJoinSubstitution([map_folder_dir, map_name])
    pbstream_path = PathJoinSubstitution([map_folder_dir, map_name])

    save_map = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        arguments=[
            '-f', PathJoinSubstitution([map_folder_dir, map_name]),
            '--free', '0.196',
            '--occ', '0.65'
            ],
        condition=IfCondition(PythonExpression(["'", map_type, "' == 'gridmap'"]))
    )


    save_pbstream = ExecuteProcess(
        condition=IfCondition(PythonExpression(["'", map_type, "' == 'pbstream'"])),
        cmd=[
            'ros2', 'service', 'call',
            '/write_state',
            'cartographer_ros_msgs/srv/WriteState',
            ['{filename: "', pbstream_path, '.pbstream", include_unfinished_submaps: true}'],
        ],
        shell=False,
        emulate_tty=True,
        output='screen',
    )
    
    save_posegraph = ExecuteProcess(
        condition=IfCondition(PythonExpression(["'", map_type, "' == 'posegraph'"])),
        cmd=[
            'ros2', 'service', 'call',
            '/slam_toolbox/serialize_map',
            'slam_toolbox/srv/SerializePoseGraph',
            ['{filename: "', map_path, '"}'],
        ],
        shell=False,
        emulate_tty=True,
        output='screen',
    )


    return LaunchDescription([
        *declared_arguments,
        save_map,
        save_pbstream,
        save_posegraph,
    ])
