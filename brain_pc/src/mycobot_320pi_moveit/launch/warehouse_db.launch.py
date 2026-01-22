from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mycobot_320pi", package_name="mycobot_320pi_moveit").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
