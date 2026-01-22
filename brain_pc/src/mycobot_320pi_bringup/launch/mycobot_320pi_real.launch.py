import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    robot_2030a_description_dir = get_package_share_directory('mycobot_320pi_description')

    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("mycobot_320pi_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("mycobot_320pi_moveit"),
                "launch",
                "move_group.launch.py"
            ),
            launch_arguments={"is_sim": "False", "default_planning_pipeline": "ompl"}.items()
        )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(robot_2030a_description_dir, 'rviz', 'display_moveit.rviz')],
    )
    
  
    
    return LaunchDescription([
        controller,
        moveit,
        rviz_node,
    ])