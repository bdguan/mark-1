import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- YOUR IDS ---
    wrist_serial = '207122078535'
    global_mxid  = '194430109157715A00' 
    # ----------------

    return LaunchDescription([
        
        # 1. Wrist Camera (Intel RealSense)
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera_wrist',
            namespace='camera_wrist',
            parameters=[{
                'serial_no': wrist_serial,
                'camera_name': 'camera_wrist',
                'base_frame_id': 'd435i_camera_link',
                'depth_module.profile': '640x480x15',
                'rgb_camera.profile': '640x480x15',
                'align_depth.enable': True,
            }],
            output='screen'
        ),

        # 2. Global Camera (OAK-D)
        # We pass specific parameters to fix the "Socket 0" error if possible
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')
            ),
            launch_arguments={
                'mxid': global_mxid,
                'camera_model': 'OAK-D',
                'parent_frame': 'camera_global_link',
                'cam_pos_x': '0.0', 'cam_pos_y': '0.0', 'cam_pos_z': '0.0',
                'cam_roll': '0.0', 'cam_pitch': '0.0', 'cam_yaw': '0.0',
                # If RGB fails, we might need to rely on stereo, but let's try standard first
            }.items()
        )
    ])
