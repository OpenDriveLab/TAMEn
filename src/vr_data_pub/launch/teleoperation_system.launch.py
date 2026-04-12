from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vr_data_pub',
            executable='vr_data_pub',
            parameters=[{'server_port': 8000}],  # Set the server_port parameter
            name='vr_socket_server'
        ),
        Node(
            package='vr_data_pub',
            executable='vr_data_distributer',
            name='vr_data_distributor'
        ),
        Node(
            package='k1_robot',
            namespace='/left_arm',
            parameters=[{'robot_ip': '192.168.2.222'}],  # Set the robot_ip parameter
            executable='robot_interface',
            name='jaka_robot_driver'
        ),
        Node(
            package='k1_robot',
            namespace='/right_arm',
            parameters=[{'robot_ip': '192.168.2.223'}],  # Set the robot_ip parameter
            executable='robot_interface',
            name='jaka_robot_driver'
        ),
        Node(
            package='vr_data_pub',
            namespace='/left_arm',
            executable='vr_robot_pose_converter',
            name='vr_robot_pose_converter'
        ),
        Node(
            package='vr_data_pub',
            namespace='/right_arm',
            executable='vr_robot_pose_converter',
            name='vr_robot_pose_converter'
        )#,
        # can not exit automatically, use terminal now
#        Node(
#            package='vr_data_pub',
#            executable='teleoperation_control',
#            name='vr_state_machine_node'
#        ),
        # remap topic template
#        Node(
#            package='turtlesim',
#            executable='mimic',
#            name='mimic',
#            remappings=[
#                ('/input/pose', '/turtlesim1/turtle1/pose'),
#                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
#            ]
#        )
    ])