from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_perception = True
    moveit_builder = MoveItConfigsBuilder("kai01_robot", package_name="moveit_config")

    if use_perception:
        sensors_3d_path = os.path.join(
            get_package_share_directory("kai01_bringup"),
            "config",
            "sensors_3d.yaml",
        )
        moveit_builder = moveit_builder.sensors_3d(sensors_3d_path)

    moveit_config = moveit_builder.to_moveit_configs()
    return generate_move_group_launch(moveit_config)
