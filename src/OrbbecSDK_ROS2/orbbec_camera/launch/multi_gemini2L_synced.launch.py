import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import time


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    config_file_dir = os.path.join(package_dir, 'config')
    config_file_path = os.path.join(config_file_dir, 'gemini2L_params.yaml')
    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'usb_port': '4-1',
            'device_num': '5',
            'sync_mode': 'PRIMARY',
            'config_file_path': config_file_path,
        }.items()
    )
    time.sleep(2)

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'usb_port': '4-3.1',
            'device_num': '5',
            'sync_mode': 'SECONDARY',  #  TODO: make sure different mode 
            'config_file_path': config_file_path,
        }.items()
    )
    time.sleep(2)

    front_camera = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(launch_file_dir, 'gemini2L.launch.py')
    ),
    launch_arguments={
        'camera_name': 'front_camera',
        'usb_port': '4-2',
        'device_num': '5',
        'sync_mode': 'SECONDARY', 
        'config_file_path': config_file_path,
    }.items()
    )
    time.sleep(2)

    left_fixed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_fixed_camera',
            'usb_port': '4-4',
            'device_num': '5',
            'sync_mode': 'SECONDARY',  #  TODO: make sure different mode 
            'config_file_path': config_file_path,
        }.items()
    )
    time.sleep(2)

    right_fixed_camera = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(launch_file_dir, 'gemini2L.launch.py')
    ),
    launch_arguments={
        'camera_name': 'right_fixed_camera',
        'usb_port': '4-3.2',
        'device_num': '5',
        'sync_mode': 'SECONDARY', 
        'config_file_path': config_file_path,
    }.items()
    )


    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription([
        GroupAction([left_camera]),
        GroupAction([right_camera]),
        GroupAction([left_fixed_camera]),
        GroupAction([right_fixed_camera]),
        TimerAction(period=3.0, actions=[GroupAction([front_camera])]), # The primary camera should be launched at last

    ])

    return ld
