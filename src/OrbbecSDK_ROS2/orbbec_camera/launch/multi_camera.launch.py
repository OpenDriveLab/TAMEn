from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'head_camera',
            'usb_port': '4-2',
            'device_num': '3',
            'sync_mode': 'standalone'
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'usb_port': '4-3',
            'device_num': '3',
            'sync_mode': 'standalone'
        }.items()
    )
    
    launch3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'usb_port': '4-4',
            'device_num': '3',
            'sync_mode': 'standalone'
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
        GroupAction([launch3_include]),
    ])

    return ld
