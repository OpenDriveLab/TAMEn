import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from k1_msgs.srv import JointMove, LinearMove, KineInverse7#, Rotm2rpy
from k1_robot import jkrc
import math

import numpy as np

class JakaRobotDriver(Node):
    def __init__(self):
        super().__init__('jaka_robot_driver')
        
        # Publishers for joint states and current TCP position
        self.publisher_joint_states = self.create_publisher(JointState, 'joint_states', 10)
        self.publisher_joint_states_vel = self.create_publisher(JointState, 'joint_states_vel', 10)
        self.publisher_tcp_position = self.create_publisher(PoseStamped, 'cur_tcp_pose', 10)
        self.publisher_gripper_val = self.create_publisher(JointState, 'cur_gripper_val', 10)
        self.publisher_raw_torque_sensor_val = self.create_publisher(JointState, 'raw_torque_sensor_val', 10)

        timer_period = 0.008  # seconds (8 ms)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Services for joint move, linear move, inverse kinematics, and servo move enable
        self.joint_move_service = self.create_service(JointMove, 'joint_move', self.joint_move_callback)
        self.linear_move_service = self.create_service(LinearMove, 'linear_move', self.linear_move_callback)
        self.kine_inverse_service = self.create_service(KineInverse7, 'kine_inverse7', self.kine_inverse_callback)
        # self.rotm2rpy_service = self.create_service(Rotm2rpy, 'rotm2rpy', self.rotm2rpy_callback)
        self.servo_move_enable_service = self.create_service(SetBool, 'servo_move_enable', self.servo_move_enable_callback)

        # Subscriber for servo_j7 commands
        self.subscription_servo_j = self.create_subscription(
            JointState,
            'robot_command_servo_j',
            self.servo_j7_callback,
            10)

        self.subscription_servo_p = self.create_subscription(
            JointState,
            'robot_command_servo_p',
            self.servo_p_callback,
            10)


        # Subscriber for gripper value commands
        self.subscription_servo_j = self.create_subscription(
            JointState,
            'gripper_command_val',
            self.gripper_val_callback,
            10)

        self.declare_parameter('robot_ip', "192.168.2.222")
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        # self.robot = jkrc.RC("192.168.2.222")  # Replace with the actual IP address
        self.robot = jkrc.RC(robot_ip)  # Replace with the actual IP address
        self.login()
        self.enable_robot()

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

        self.servo_move_enabled = False  # Track the state of servo_move_enable

    def login(self):
        ret = self.robot.login()
        if ret[0] != 0:
            self.get_logger().error(f'Failed to login. Error code: {ret[0]}')
            exit(1)

    def enable_robot(self):
        ret = self.robot.power_on()  # Power-on operation
        if ret[0] != 0:
            self.get_logger().error(f'Failed to power on robot. Error code: {ret[0]}')
            exit(1)
        ret = self.robot.enable_robot()
        if ret[0] != 0:
            self.get_logger().error(f'Failed to enable robot. Error code: {ret[0]}')
            exit(1)

    def shutdown(self):
        self.robot.logout()

    def timer_callback(self):
        # Publish joint states
        current_time = self.get_clock().now().to_msg()
        msg_joint_state = JointState()
        msg_joint_state.header.stamp = current_time
        msg_joint_state.name = self.joint_names

        msg_tcp_pose = PoseStamped()
        msg_tcp_pose.header.stamp = current_time

        msg_gripper = JointState()
        msg_gripper.header.stamp = current_time
        msg_gripper.name = self.joint_names

        msg_raw_torque_sensor = JointState()
        msg_raw_torque_sensor.header.stamp = current_time
        msg_raw_torque_sensor.name = self.joint_names

        msg_joint_state_vel = JointState()
        msg_joint_state_vel.header.stamp = current_time
        msg_joint_state_vel.name = self.joint_names

        ret_robot_status = self.robot.get_robot_status()
        if ret_robot_status[0] == 0:
            # self.get_logger().info(f'get robot status : {np.array(ret_robot_status[1][20][5])[:,3]}')
            msg_joint_state.position = ret_robot_status[1][19]
            
            msg_raw_torque_sensor.position = ret_robot_status[1][21][6]
            self.get_logger().warn(f'robot status torque sensor. {msg_raw_torque_sensor.position}')
            # temp_var = ret_robot_status[1][21][6]
            # self.get_logger().warn(f'robot status torque sensor. {temp_var}')

            msg_tcp_pose.pose.position.x = ret_robot_status[1][18][0]
            msg_tcp_pose.pose.position.y = ret_robot_status[1][18][1]
            msg_tcp_pose.pose.position.z = ret_robot_status[1][18][2]
            # Convert rotation angles to a quaternion
            quaternion = self.rpy_to_quaternion(ret_robot_status[1][18][3:])
            msg_tcp_pose.pose.orientation.w = quaternion[0]
            msg_tcp_pose.pose.orientation.x = quaternion[1]
            msg_tcp_pose.pose.orientation.y = quaternion[2]
            msg_tcp_pose.pose.orientation.z = quaternion[3]

            msg_joint_state_vel.velocity = list(np.array(ret_robot_status[1][20][5])[:,3])

            msg_gripper.position = [(float)(ret_robot_status[1][17][2][2]/1000), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # msg_gripper.position = [float('nan') for _ in self.joint_names]
        else:
            self.get_logger().warn(f'Failed to get robot status. Error code: {ret_robot_status[0]}')

            msg_joint_state.position = [float('nan') for _ in self.joint_names]
            msg_joint_state_vel.position = [float('nan') for _ in self.joint_names]

            msg_tcp_pose.pose.position.x = float('nan')
            msg_tcp_pose.pose.position.y = float('nan')
            msg_tcp_pose.pose.position.z = float('nan')
            msg_tcp_pose.pose.orientation.w = 1.0  # Default quaternion representing no rotation
            msg_tcp_pose.pose.orientation.x = 0.0
            msg_tcp_pose.pose.orientation.y = 0.0
            msg_tcp_pose.pose.orientation.z = 0.0

            msg_gripper.position = [float('nan') for _ in self.joint_names]


        self.publisher_joint_states.publish(msg_joint_state)
        self.publisher_joint_states_vel.publish(msg_joint_state_vel)
        self.publisher_tcp_position.publish(msg_tcp_pose)
        self.publisher_gripper_val.publish(msg_gripper)

        
#        ret_joint_pos = self.robot.get_joint_position()
#        if ret_joint_pos[0] == 0:  # Joint positions fetched successfully
#            msg_joint_state.position = ret_joint_pos[1]
#        else:
#            self.get_logger().warn(f'Failed to get joint positions. Error code: {ret_joint_pos[0]}')
#            msg_joint_state.position = [float('nan') for _ in self.joint_names]
#        
#        self.publisher_joint_states.publish(msg_joint_state)
#
#        # Get and publish the TCP pose
#        msg_tcp_pose = PoseStamped()
#        msg_tcp_pose.header.stamp = current_time
#        ret_tcp_pos = self.robot.get_tcp_position()
#        if ret_tcp_pos[0] == 0:  # TCP pose fetched successfully
#            msg_tcp_pose.pose.position.x = ret_tcp_pos[1][0]
#            msg_tcp_pose.pose.position.y = ret_tcp_pos[1][1]
#            msg_tcp_pose.pose.position.z = ret_tcp_pos[1][2]
#            # Convert rotation angles to a quaternion
#            quaternion = self.rpy_to_quaternion(ret_tcp_pos[1][3:])
#            msg_tcp_pose.pose.orientation.w = quaternion[0]
#            msg_tcp_pose.pose.orientation.x = quaternion[1]
#            msg_tcp_pose.pose.orientation.y = quaternion[2]
#            msg_tcp_pose.pose.orientation.z = quaternion[3]
#        else:
#            self.get_logger().warn(f'Failed to get TCP position. Error code: {ret_tcp_pos[0]}')
#            # Fill with defaults or NaN
#            msg_tcp_pose.pose.position.x = float('nan')
#            msg_tcp_pose.pose.position.y = float('nan')
#            msg_tcp_pose.pose.position.z = float('nan')
#            msg_tcp_pose.pose.orientation.w = 1.0  # Default quaternion representing no rotation
#            msg_tcp_pose.pose.orientation.x = 0.0
#            msg_tcp_pose.pose.orientation.y = 0.0
#            msg_tcp_pose.pose.orientation.z = 0.0
#        
#        self.publisher_tcp_position.publish(msg_tcp_pose)
#

#        gripper = self.robot.get_analog_output(iotype = 2,index = 3)
#        msg_gripper = Int32()
#        if gripper[0] == 0: # Gripper opening width fetched successfully
#            msg_gripper.data = (int)(gripper[1])
#        else:
#            self.get_logger().warn(f'Failed to get gripper val. Error code: {gripper[0]}')
#            msg_gripper.data = float('nan')
#
#        self.publisher_gripper_val.publish(msg_gripper)



        # self.get_logger().info('Publishing robot states')

    def joint_move_callback(self, request, response):
        try:
            result = self.robot.joint_move7(request.joint_positions, request.move_mode, request.is_blocking, request.speed)
            response.success = int(result[0] == 0)
            response.message = "Success" if response.success else f"Error: {result[0]}"
        except Exception as e:
            response.success = 0
            response.message = str(e)
        return response

    def linear_move_callback(self, request, response):
        try:
            result = self.robot.linear_move(request.end_position, request.move_mode, request.is_blocking, request.speed)
            response.success = int(result[0] == 0)
            response.message = "Success" if response.success else f"Error: {result[0]}"
        except Exception as e:
            response.success = 0
            response.message = str(e)
        return response

    def kine_inverse_callback(self, request, response):
        try:
            result = self.robot.kine_inverse7(request.ref_joint_pos, request.cartesian_pose)
            response.success = int(result[0] == 0)
            response.message = "Success" if response.success else f"Error: {result[0]}"
            if response.success:
                response.joint_positions = result[1]
            else:
                response.joint_positions = []  # Or fill with NaNs if preferred
        except Exception as e:
            response.success = 0
            response.message = str(e)
            response.joint_positions = []
        return response

    def servo_move_enable_callback(self, request, response):
        """Service callback used to enable or disable SERVO MOVE mode."""
        try:
            enable = request.data
            result = self.robot.servo_move_enable(enable)
            self.servo_move_enabled = enable  # Update internal state
            response.success = int(result[0] == 0)
            response.message = "Enabled SERVO MOVE mode" if enable else "Disabled SERVO MOVE mode"
            if not response.success:
                response.message = f"Error: {result[0]}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def servo_j7_callback(self, msg):
        """Callback that handles messages from /robot_command_servo_j."""
        if not self.servo_move_enabled:
            self.get_logger().warn("Servo Move is not enabled. Please enable it first.")
            return

        try:
            joint_positions = list(msg.position)  # Ensure we have the correct data type
            move_mode = 0 if len(msg.name) == 0 else 1  # No joint names means absolute motion

            # Validate the joint-position payload
            if len(joint_positions) != 7:
                self.get_logger().error("Invalid number of joint positions provided. Expected 7, got {}".format(len(joint_positions)))
                return

            # Call the servo_j7 interface
            result = self.robot.servo_j7(joint_positions, move_mode)

            if result[0] == 0:
                self.get_logger().info("Successfully sent servo_j7 command.")
            else:
                self.get_logger().error(f"Failed to send servo_j7 command. Error code: {result[0]}")
        except Exception as e:
            self.get_logger().error(f"Exception occurred while processing servo_j7 command: {str(e)}")

    def servo_p_callback(self, msg):
        """Callback that handles messages from /robot_command_servo_j."""
        if not self.servo_move_enabled:
            self.get_logger().warn("Servo Move is not enabled. Please enable it first.")
            return

        try:
            joint_positions = list(msg.position)  # Ensure we have the correct data type
            move_mode = 0 if len(msg.name) == 0 else 1  # No joint names means absolute motion

            # Validate the joint-position payload
            if len(joint_positions) != 7:
                self.get_logger().error("Invalid number of joint positions provided. Expected 7, got {}".format(len(joint_positions)))
                return

            # Call the servo_j7 interface
            result = self.robot.servo_j7(joint_positions, move_mode)

            if result[0] == 0:
                self.get_logger().info("Successfully sent servo_j7 command.")
            else:
                self.get_logger().error(f"Failed to send servo_j7 command. Error code: {result[0]}")
        except Exception as e:
            self.get_logger().error(f"Exception occurred while processing servo_j7 command: {str(e)}")
    
    def gripper_val_callback(self, msg):
        try:
            gripper_val_array = msg.position
            result = self.robot.set_analog_output(iotype=2, index=3,value = (int)(gripper_val_array[0]*1000))
            if result[0] == 0:
                self.get_logger().info("Successfully sent gripper val command.")
            else:
                self.get_logger().error(f"Failed to send gripper val command. Error code: {result[0]}")
        except Exception as e:
            self.get_logger().error(f"Exception occurred while processing gripper val command: {str(e)}")

    def rpy_to_quaternion(self, rpy):
        """ Convert RPY angles to Quaternion """
        roll, pitch, yaw = rpy
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (w, x, y, z)

def main(args=None):
    rclpy.init(args=args)
    driver = JakaRobotDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()