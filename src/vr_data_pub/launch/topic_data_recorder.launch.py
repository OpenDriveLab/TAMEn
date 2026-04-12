from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vr_data_pub',
            executable='topic_data_recorder',
            name='topic_data_recorder',
            parameters=[{
                'output_dir': '/root/K1-W/data',
                'file_prefix': 'data',
                'save_format': 'csv',  # csv or json
                'skip_edg_stat_without_valid_servo_p': True,
                'skip_servo_p_filtered_without_valid_servo_p': True,
                'valid_servo_p_timeout_sec': 0.5
            }]
        )
    ])
