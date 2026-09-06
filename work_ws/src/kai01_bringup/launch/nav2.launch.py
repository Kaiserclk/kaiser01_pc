import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,PythonExpression,PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    map_folder_dir = os.path.join(os.path.expanduser('~'), 'work_ws' ,'map')
    bringup_dir=os.path.join(get_package_share_directory('kai01_bringup'))
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "map",
            default_value="map.yaml",
            description="map name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value='True',
            description="if rviz is used, set to True. If False, rviz will not be launched.",
        )
    )
    #cartographer state file
    declared_arguments.append(
        DeclareLaunchArgument(
            "state_filename",
            default_value="map.pbstream",
            description="==============",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "localization_mode",
            default_value="cartographer",
            description="localization_mode ,amcl or cartographer or rtabmap",
        )
    )
    
    #slam mode
    declared_arguments.append(
        DeclareLaunchArgument(
            "slam",
            default_value="False",
            description="Start SLAM simultaneously with navigation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value='info',
            description="ROS log level: debug, info, warn, error, fatal",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_map",
            default_value="map",
            description="",
        )
    )

    # Initialize Arguments 
    map_file = PathJoinSubstitution([map_folder_dir, LaunchConfiguration("map")])
    pose_map=PathJoinSubstitution([map_folder_dir, LaunchConfiguration("pose_map")])
    state_filename_file = PathJoinSubstitution([map_folder_dir, LaunchConfiguration("state_filename")])
    rviz_config_file=os.path.join(get_package_share_directory('kai01_bringup'),'rviz','nav2.rviz')
    rviz = LaunchConfiguration("rviz")
    slam = LaunchConfiguration('slam')
    localization_mode=LaunchConfiguration("localization_mode")

    log_level=LaunchConfiguration('log_level')
    params_file=os.path.join(bringup_dir, "config","nav2_params.yaml")

    #navigation2 startup file
    nav2_launch_file = os.path.join(get_package_share_directory("nav2_bringup"), 
                                    "launch",
                                    'bringup_launch.py')
    
    #The modified startup file of navigation2 blocks the AMCL positioning function
    nav2_launch_mod = os.path.join(bringup_dir,
                                    'launch',
                                    'include',
                                    'bringup_mod.launch.py'
                                    )

    #rtabmap positioning algorithm startup file
    rtabmap_localization_launch_file=os.path.join(bringup_dir,
                                        'launch',
                                        'rtab_map.launch.py')
    
    #AMCL positioning method navigation
    amcl_localization_nav = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl' and '", slam, "' == 'False' "])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    "map": map_file,
                    "params_file": params_file,
                    "use_sim_time": "False",
                    "log_level": log_level,
                }.items(),
            )
        ]
    )

    #Cartographer positioning navigation
    cartographer_localization_nav = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'cartographer' and '", slam, "' == 'False' "])),
        actions=[
            #Start navigation2 navigation file (no global positioning)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_mod),
                launch_arguments={
                                "map": map_file,
                                "params_file": params_file,
                                "use_sim_time": "False",
                                "log_level": log_level,
                                }.items()),
            Node(
                package =   'cartographer_ros',
                executable = 'cartographer_node',
                parameters = [{'use_sim_time': False}],
                arguments = [
                            '-configuration_directory', os.path.join(get_package_share_directory('kai01_bringup'),'config'),
                            '-configuration_basename', 'cartographer_2d_localization.lua',
                            '-load_state_filename', state_filename_file],
                remappings = [('imu', '/imu/data_raw')],
                output = 'screen'
                ),
            Node(
                package = 'cartographer_ros',
                executable = 'cartographer_occupancy_grid_node',
                parameters = [
                    {'use_sim_time': False},
                    {'resolution': 0.01}],
                arguments = ['--include_unfrozen_submaps=false'],
                )
        ]
    )

    #rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file,'--ros-args','--log-level','warn'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", localization_mode, "' != 'rtabmap' and '", rviz, "' == 'True' "]))
    )   
    
    slam_toolbox_localization_nav = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'slam_toolbox' and '", slam, "' == 'False' "])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_mod),
                launch_arguments={
                                "map": map_file,
                                "params_file": params_file,
                                "use_sim_time": "False",
                                "log_level": log_level,
                                }.items()),
            Node(
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                  os.path.join(bringup_dir,'config','mapper_params_localization.yaml'),
                  {'use_sim_time': False},
                  {'map_file_name': pose_map}
                ],
                arguments=['--ros-args','--log-level','warn'],
                output='screen')
        ]
    )
    #Navigate while building the map
    nav_and_slam = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl' and '", slam, "' == 'True' "])),
        actions=[
            # Navigation using AMCL positioning method
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": "False",
                    "params_file": params_file,
                    'slam': 'True',
                }.items(),
            )
        ]
    )

    #rtabmap positioning navigation
    rtabmap_localization_nav = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'rtabmap' and '", slam, "' == 'False' "])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_mod),
                launch_arguments={
                                "map": map_file,
                                "params_file": params_file,
                                "use_sim_time": "False",
                                }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_localization_launch_file),
                launch_arguments={
                    "mode": "localization",
                }.items(),
            )
        ]
    )
    
    return LaunchDescription(
        [
            *declared_arguments,                                          
            amcl_localization_nav,              
            cartographer_localization_nav,      
            slam_toolbox_localization_nav,
            rtabmap_localization_nav,           
            nav_and_slam,
            rviz_node                     
        ]
    )


