from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vr_data_pub',
            executable='vr_data_pub',
            parameters=[{'server_port': 8018}],  # Set the server_port parameter
            name='vr_socket_server'
        ),
        Node(
            package='vr_data_pub',
            executable='vr_data_distributer',
            name='vr_data_distributor'
        ),
        Node(
            package='robohub',
            # namespace='/left_arm',
            parameters=[{
                'robot_ip': '10.5.5.100',
                'robot_running_mode':'teleop',  #teleop or act_exe or test
                'edg_local_ip': '10.5.5.104',  # Local EDG host IP (important)
                'edg_port': 10011  # EDG port (avoid conflicts)
            }],
            executable='jaka_driver_node',
            name='jaka_robot_driver'
        ),   
        # Node(
        #     package='jakazu_body_control',
        #     # namespace='/left_arm',
        #     parameters=[{'robot_ip': '172.30.95.93'}],  # Set robot_ip for the left arm, index 0
        #     executable='jakazu_body_control',
        #     name='jakazu_body_control'
        # ),
        Node(
            package='k1_robot',
            # namespace='/left_arm',
            executable='control_gripper',
            name='control_gripper'
        ),
#        Node(
#            package='robohub',
#            namespace='/right_arm',
#            parameters=[{'robot_ip': '172.30.95.233'},{'robot_arm_index': 1}],  # Set robot_ip for the right arm, index 1
#            executable='jaka_driver_node',
#            name='jaka_robot_driver'
#        ),
        Node(
            package='vr_data_pub',
            namespace='/left_arm',
            parameters = [{'robot_name':'left'}],
            executable='vr_robot_pose_converter_30degree_servo_p',
            name='vr_robot_pose_converter'
        ),
        # Left-arm interpolation node: upsample 60-90 Hz data to a stable 1000 Hz
        Node(
            package='vr_data_pub',
            namespace='/left_arm',
            parameters=[{
                'output_frequency': 1000.0,  # Target output frequency: 1000 Hz
                'buffer_size': 10,  # Buffer size to absorb input-rate jitter
                'max_extrapolation_time': 0.05  # Maximum extrapolation window: 50 ms
            }],
            executable='pose_interpolator',
            name='pose_interpolator'
        ),
        Node(
            package='vr_data_pub',
            namespace='/right_arm',
            parameters=[{'robot_name': 'right'}],
            executable='vr_robot_pose_converter_30degree_servo_p',
            name='vr_robot_pose_converter'
        ),
        # Right-arm interpolation node: upsample 60-90 Hz data to a stable 1000 Hz
        Node(
            package='vr_data_pub',
            namespace='/right_arm',
            parameters=[{
                'output_frequency': 125.0,  # Target output frequency
                'buffer_size': 10,  # Buffer size to absorb input-rate jitter
                'max_extrapolation_time': 0.05  # Maximum extrapolation window: 50 ms
            }],
            executable='pose_interpolator',
            name='pose_interpolator'
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
