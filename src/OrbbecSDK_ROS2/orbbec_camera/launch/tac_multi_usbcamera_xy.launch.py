##############################
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare parameters for multiple USB camera devices
    # front_camera_device_arg = DeclareLaunchArgument(
    #     'front_camera_device',
    #     default_value='/dev/video0',
    #     description='Front camera device path (e.g., /dev/video0)'
    # )
    
    # Add more parameters here if additional cameras are needed
    left_camera_device_arg = DeclareLaunchArgument(
        'left_camera_device',
        default_value='/dev/video4',
        description='Left camera device path (e.g., /dev/video1)'
    )
    
    right_camera_device_arg = DeclareLaunchArgument(
        'right_camera_device',
        default_value='/dev/video6',
        description='Right camera device path (e.g., /dev/video2)'
    )
    
    # Tactile camera device parameters
    right_tactile_camera_device_arg = DeclareLaunchArgument(
        'right_tactile_camera_device',
        default_value='/dev/video0',
        description='Right tactile camera device path (e.g., /dev/video1)'
    )
    
    left_tactile_camera_device_arg = DeclareLaunchArgument(
        'left_tactile_camera_device',
        default_value='/dev/video2',
        description='Left tactile camera device path (e.g., /dev/video3)'
    )

    # front_camera node - publishes to /front_camera/color/image_raw
    # front_camera_node = Node(
    #     package='orbbec_camera',
    #     executable='usb_camera_publisher.py',
    #     name='front_camera_publisher',
    #     namespace='front_camera',
    #     parameters=[{
    #         'device_path': LaunchConfiguration('front_camera_device'),
    #         'frame_id': 'front_camera_frame',
    #         'fps': 30.0,
    #         'width': 640,
    #         'height': 480,
    #     }],
    #     output='screen'
    # )
    
    # Add more nodes here if additional cameras are needed
    left_camera_node = Node(
        package='orbbec_camera',
        executable='usb_camera_publisher.py',
        name='left_camera_publisher',
        namespace='left_camera',
        parameters=[{
            'device_path': LaunchConfiguration('left_camera_device'),
            'frame_id': 'left_camera_frame',
            'fps': 30.0,
            'width': 640,
            'height': 480,
        }],
        output='screen'
    )
    
    right_camera_node = Node(
        package='orbbec_camera',
        executable='usb_camera_publisher.py',
        name='right_camera_publisher',
        namespace='right_camera',
        parameters=[{
            'device_path': LaunchConfiguration('right_camera_device'),
            'frame_id': 'right_camera_frame',
            'fps': 30.0,
            'width': 640,
            'height': 480,
        }],
        output='screen'
    )
    
    # Tactile camera node - publishes to /right_tactile_camera/color/image_raw
    right_tactile_camera_node = Node(
        package='orbbec_camera',
        executable='usb_camera_publisher.py',
        name='right_tactile_camera_publisher',
        namespace='right_tactile_camera',
        parameters=[{
            'device_path': LaunchConfiguration('right_tactile_camera_device'),
            'frame_id': 'right_tactile_camera_frame',
            'fps': 30.0,
            'width': 640,
            'height': 480,
        }],
        output='screen'
    )
    
    # Tactile camera node - publishes to /left_tactile_camera/color/image_raw
    left_tactile_camera_node = Node(
        package='orbbec_camera',
        executable='usb_camera_publisher.py',
        name='left_tactile_camera_publisher',
        namespace='left_tactile_camera',
        parameters=[{
            'device_path': LaunchConfiguration('left_tactile_camera_device'),
            'frame_id': 'left_tactile_camera_frame',
            'fps': 30.0,
            'width': 640,
            'height': 480,
        }],
        output='screen'
    )

    # Launch description
    ld = LaunchDescription([
        # front_camera_device_arg,
        # front_camera_node,
        # Vision camera arguments and nodes
        left_camera_device_arg,
        right_camera_device_arg,
        left_camera_node,
        right_camera_node,
        # Tactile camera arguments and nodes
        right_tactile_camera_device_arg,
        left_tactile_camera_device_arg,
        right_tactile_camera_node,
        left_tactile_camera_node,
    ])

    return ld
