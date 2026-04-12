import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from transitions import Machine, State
from k1_msgs.srv import JointMove

class StateMachineNode(Node):

    states = ['DEFAULT', 'OPERATIONAL']

    def __init__(self):
        super().__init__('state_machine_node')
        self.machine = Machine(model=self, states=StateMachineNode.states, initial='DEFAULT')
        self.machine.add_transition(trigger='to_operational', source='DEFAULT', dest='OPERATIONAL', after='activate_services')
        self.machine.add_transition(trigger='to_default', source='OPERATIONAL', before='move_to_default_pose', dest='DEFAULT', after='deactivate_all_services')
        self.machine.add_transition(trigger='shutdown', source='*', dest='DEFAULT', before='deactivate_all_services', after='node_shutdown')

        # Initialize service service_clients for nodes A-D
        self.service_clients = {
            'start_left_arm_servo_mode': self.create_client(SetBool, '/left_arm/servo_move_enable'),
            'start_left_arm_vr_robot_pose_converter': self.create_client(SetBool, '/left_arm/vr_robot_pose_converter'),
            'start_right_arm_servo_mode': self.create_client(SetBool, '/right_arm/servo_move_enable'),
            'start_right_arm_vr_robot_pose_converter': self.create_client(SetBool, '/right_arm/vr_robot_pose_converter'),
            # 'B': self.create_client(SetBool, '/node_B/set_bool'),
            # 'C': self.create_client(SetBool, '/node_C/set_bool'),
            # 'D': self.create_client(SetBool, '/node_D/set_bool')
        }

        # Initialize service service_clients for nodes E and F
        self.special_service_clients = {
            'start_recording': self.create_client(SetBool, 'start_recording'),
            'save_data': self.create_client(SetBool, 'save_data'),
        }

        self.publisher_left_command_gripper_val = self.create_publisher(JointState, 'left_arm/gripper_command_val', 10)
        self.publisher_right_command_gripper_val = self.create_publisher(JointState, 'right_arm/gripper_command_val', 10)

        self.left_arm_joint_move_client = self.create_client(JointMove, 'left_arm/joint_move')
        self.right_arm_joint_move_client = self.create_client(JointMove, 'right_arm/joint_move')


        self.subscriptions_list = [
            # self.create_subscription(String, '/vr_left_side_button', lambda msg: self.to_operational() if msg.data == 'LG=T' else None, 10),
            # self.create_subscription(String, '/vr_right_side_button', lambda msg: self.to_operational() if msg.data == 'RG=T' else None, 10),
            # self.create_subscription(String, '/vr_b_button', lambda msg: self.to_default() if msg.data == 'B=T' else None, 10),
            # self.create_subscription(String, '/vr_y_button', lambda msg: self.shutdown() if msg.data == 'Y=T' else None, 10),
            # self.create_subscription(String, '/vr_x_button', self.x_button_callback, 10),  # X button callback
            # self.create_subscription(String, '/vr_a_button', self.a_button_callback, 10)   # A button callback
            self.create_subscription(String, '/vr_left_side_button', self.left_side_button_callback, 10),
            self.create_subscription(String, '/vr_right_side_button', self.right_side_button_callback, 10),
            self.create_subscription(String, '/vr_left_front_button', self.left_front_button_callback, 10),
            self.create_subscription(String, '/vr_right_front_button', self.right_front_button_callback, 10),
            self.create_subscription(String, '/vr_b_button', self.b_button_callback, 10),
            self.create_subscription(String, '/vr_y_button', self.y_button_callback, 10),
            self.create_subscription(String, '/vr_x_button', self.x_button_callback, 10),  # X button callback
            self.create_subscription(String, '/vr_a_button', self.a_button_callback, 10)   # A button callback
        ]
        
        self.get_logger().info('State machine node has been started.')

    def activate_services(self):
        self.get_logger().info('Activating services.')
        for client in self.service_clients.values():
            if client.service_is_ready():
                request = SetBool.Request()
                request.data = True
                future = client.call_async(request)

    def move_to_default_pose(self):
        self.get_logger().info('Move to default pose.')
        request = JointMove.Request()
        request.move_mode = 0 # absolute move
        request.is_blocking = False
        request.speed = 1.0
        # left arm value
        # request.joint_positions =[-0.9789622807874625, -1.2452436262852262, 1.2387750528597778, 
                                #  -1.3914963805868434, 0.5660937859753931, 1.1651303101928812, 0.2510335583264105] 
        request.joint_positions =[-0.9789622807874625, -1.2452436262852262, 1.2387750528597778, 
                                  -1.3914963805868434, 0.5660937859753931, 1.1651303101928812, 1.8] 
        future = self.left_arm_joint_move_client.call_async(request)
        # right arm value
        # request.joint_positions = [1.2114602413063924, -1.0031153996588744, -1.3729047724531143, 
                                #   -1.653521738379849, -0.6401294555159845, -1.1206928043794315, -1.3373245160373455]
        request.joint_positions = [1.2114602413063924, -1.0031153996588744, -1.3729047724531143, 
                                   -1.653521738379849, -0.6401294555159845, -1.1206928043794315, 0.3]
        future = self.right_arm_joint_move_client.call_async(request)


    def deactivate_services(self, service_clients=None):
        if service_clients is None:
            service_clients = self.service_clients.values()

        self.get_logger().info('Deactivating services.')
        for client in service_clients:
            if client.service_is_ready():
                request = SetBool.Request()
                request.data = False
                future = client.call_async(request)

    def deactivate_all_services(self):
        self.get_logger().info('stop all services.')
        self.deactivate_services(list(self.service_clients.values()) + list(self.special_service_clients.values()))
    
    def left_side_button_callback(self, msg):
        if msg.data == 'LG=T' and self.state != 'OPERATIONAL':
            self.get_logger().info('left trans to operational.')
            self.to_operational()

    def right_side_button_callback(self, msg):
        if msg.data == 'RG=T' and self.state != 'OPERATIONAL':
            self.get_logger().info('right trans to operational.')
            self.to_operational()

    def left_front_button_callback(self, msg):
        if self.state == 'OPERATIONAL':
            current_time = self.get_clock().now().to_msg()
            msg_gripper = JointState()
            msg_gripper.header.stamp = current_time
            if msg.data == 'LT=T':
                msg_gripper.position= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.publisher_left_command_gripper_val.publish(msg_gripper)
                self.get_logger().info('invoke left gripper close')
            if msg.data == 'LT=F':
                msg_gripper.position= [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.publisher_left_command_gripper_val.publish(msg_gripper)
                self.get_logger().info('invoke left gripper open.')

    def right_front_button_callback(self, msg):
        if self.state == 'OPERATIONAL':
            current_time = self.get_clock().now().to_msg()
            msg_gripper = JointState()
            msg_gripper.header.stamp = current_time
            if msg.data == 'RT=T':
                msg_gripper.position= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.publisher_right_command_gripper_val.publish(msg_gripper)
                self.get_logger().info('invoke right gripper close')
            if msg.data == 'RT=F':
                msg_gripper.position= [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.publisher_right_command_gripper_val.publish(msg_gripper)
                self.get_logger().info('invoke right gripper open.')

    def b_button_callback(self, msg):
        if self.state != 'DEFAULT' and msg.data == 'B=T':
            self.get_logger().info('trans to default.')
            self.to_default()

    def y_button_callback(self, msg):
        if msg.data == 'Y=T':
            self.get_logger().info('shutdown node.')
            self.shutdown()

    def x_button_callback(self, msg):
        if self.state == 'OPERATIONAL' and msg.data == 'X=T':
            self.get_logger().info('call service record.')
            self.call_service('start_recording')

    def a_button_callback(self, msg):
        if self.state == 'OPERATIONAL' and msg.data == 'A=T':
            self.get_logger().info('call service write to disk.')
            self.call_service('save_data')

    def call_service(self, service_name):
        if service_name in self.special_service_clients and self.special_service_clients[service_name].service_is_ready():
            request = SetBool.Request()
            request.data = True
            future = self.special_service_clients[service_name].call_async(request)

    def node_shutdown(self):
        self.get_logger().info('Shutting down the node.')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()
    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the node is properly shut down even on keyboard interrupt
        state_machine_node.shutdown()

if __name__ == '__main__':
    main()