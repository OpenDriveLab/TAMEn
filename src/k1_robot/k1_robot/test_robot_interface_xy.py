import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from k1_msgs.srv import JointMove, LinearMove, KineInverse7
from std_srvs.srv import SetBool
import sys
import time
import math

class JakaRobotClient(Node):
    def __init__(self):
        super().__init__('jaka_robot_client')

        # Clients for services
        self.joint_move_client = self.create_client(JointMove, 'joint_move')
        self.linear_move_client = self.create_client(LinearMove, 'linear_move')
        self.kine_inverse_client = self.create_client(KineInverse7, 'kine_inverse7')
        self.servo_move_enable_client = self.create_client(SetBool, 'left_arm/servo_move_enable')

        # Publisher to send servo_j7 commands
        self.publisher_servo_j = self.create_publisher(JointState, 'robot_command_servo_j', 10)
        self.publisher_servo_p = self.create_publisher(JointState, 'robot_command_servo_p', 10)
        
        # Wait for the services to be available
        while not self.joint_move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service joint_move not available, waiting again...')
        # while not self.linear_move_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service linear_move not available, waiting again...')
        # while not self.kine_inverse_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service kine_inverse7 not available, waiting again...')
        # while not self.servo_move_enable_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service servo_move_enable not available, waiting again...')

    def call_joint_move(self, robot_arm_index, joint_positions, move_mode, is_blocking, speed):
        request = JointMove.Request()
        request.robot_arm_index = robot_arm_index
        request.joint_positions = joint_positions
        request.move_mode = move_mode
        request.is_blocking = is_blocking
        request.speed = speed
        future = self.joint_move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_linear_move(self, end_position, move_mode, is_blocking, speed):
        request = LinearMove.Request()
        request.end_position = end_position
        request.move_mode = move_mode
        request.is_blocking = is_blocking
        request.speed = speed
        future = self.linear_move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_kine_inverse7(self, ref_joint_pos, cartesian_pose):
        request = KineInverse7.Request()
        request.ref_joint_pos = ref_joint_pos
        request.cartesian_pose = cartesian_pose
        future = self.kine_inverse_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_servo_move_enable(self, enable):
        request = SetBool.Request()
        request.data = enable
        future = self.servo_move_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def publish_servo_j7_command(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        msg.position = joint_positions
        self.publisher_servo_j.publish(msg)

    def publish_servo_p_command(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']   # abs without name, relative with name
        msg.position = joint_positions
        self.publisher_servo_p.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JakaRobotClient()
    action_publisher = Node('action_publisher')
    left_gripper_pub = action_publisher.create_publisher(JointState, '/left_gripper/set_pos', 10)
    right_gripper_pub = action_publisher.create_publisher(JointState, '/right_gripper/set_pos', 10)
    rclpy.spin_once(action_publisher, timeout_sec=1)
    for i in range(100):
        left_gripper_cmd = JointState()
        right_gripper_cmd = JointState()
        left_gripper_cmd.position = [float(sys.argv[1])]
        right_gripper_cmd.position = [float(sys.argv[1])]
        left_gripper_pub.publish(left_gripper_cmd)
        right_gripper_pub.publish(right_gripper_cmd)
    try:
        # Example usage of the client methods
        # Replace these calls with your own logic or command-line arguments
        print("before")
        response = node.call_joint_move(-1, [ -1.2066159250737598
, -0.9925338390241354
, -4.320702189652122
, -1.2612447306611823
, 0.4046371337823654
, -1.2928526434147996
, 3.146898454515856
, -2.2674095911433936
, 1.4171549927418359
, 1.516464227180313
, -1.029238113193576
, -3.312722186747837
, -1.3017712758924906
, 0.44142867441440586], 0, True, 1.0)
        time.sleep(1)
        print("after")

        response = node.call_servo_move_enable(False)
        print(f"Servo Move Enable Response: {response.message}")


    except KeyboardInterrupt:
        pass

    finally:
        action_publisher.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
