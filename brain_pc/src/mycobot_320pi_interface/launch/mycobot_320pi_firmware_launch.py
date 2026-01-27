from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_320pi_interface')
    
    params_file = PathJoinSubstitution([
        pkg_share, 'config', 'params.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='mycobot_320pi_interface',  
            executable='mycobot_320pi_interface', 
            name='mycobot_320pi_interface',
            parameters=[params_file], 
            output='screen'
        )
    ])