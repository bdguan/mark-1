import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Headless bringup: controller + move_group, no RViz."""

    is_sim = LaunchConfiguration("is_sim")

    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="False")

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mycobot_320pi_controller"),
            "launch",
            "controller.launch.py",
        ),
        launch_arguments={"is_sim": "False"}.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder("mycobot_320pi", package_name="mycobot_320pi_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("mycobot_320pi_description"),
                "urdf",
                "mycobot_320pi.urdf.xacro",
            )
        )
        .robot_description_semantic(file_path="config/mycobot_320pi.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    firmware_bridge = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mycobot_320pi_interface"),
            "launch",
            "mycobot_320pi_firmware_launch.py",
        ),
    )

    return LaunchDescription(
        [
            is_sim_arg,
            controller,
            move_group_node,
            firmware_bridge,
        ]
    )
