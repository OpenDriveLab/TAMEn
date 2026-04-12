import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from transitions import Machine, State
from k1_msgs.srv import JointMove
import math
import time

class StateMachineNode(Node):

    states = ['DEFAULT', 'OPERATIONAL']

    def __init__(self):
        super().__init__('state_machine_node')
        self.machine = Machine(model=self, states=StateMachineNode.states, initial='DEFAULT')
        self.machine.add_transition(trigger='to_operational', source='DEFAULT', dest='OPERATIONAL', after='activate_services')
        self.machine.add_transition(trigger='to_default', source='OPERATIONAL', before='move_to_default_pose', dest='DEFAULT', after='deactivate_all_services')
        self.machine.add_transition(trigger='shutdown', source='*', dest='DEFAULT', before='deactivate_all_services', after='node_shutdown')


        # force mode service requires start in advance
        self.pre_service_clients = {
            'start_force_control_mode_enable': self.create_client(SetBool, '/force_control_mode_enable'),
        }


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

        self.publisher_left_command_gripper_val = self.create_publisher(JointState, 'left_gripper/set_pos', 10)
        self.publisher_right_command_gripper_val = self.create_publisher(JointState, 'right_gripper/set_pos', 10)
        # Extra velocity publishers used to control open/close speed
        self.publisher_left_command_gripper_vel = self.create_publisher(Int32, 'left_gripper/set_vel', 10)
        self.publisher_right_command_gripper_vel = self.create_publisher(Int32, 'right_gripper/set_vel', 10)
        # Default gripper speed (smaller values mean slower motion)
        self.gripper_vel_open = 10
        self.gripper_vel_close = 10
        # self.publisher_left_command_gripper_val = self.create_publisher(Int32, 'left_gripper/set_pos', 10)
        # self.publisher_right_command_gripper_val = self.create_publisher(Int32, 'right_gripper/set_pos', 10)

        self.dual_arm_joint_move_client = self.create_client(JointMove, 'joint_move')
        # self.right_arm_joint_move_client = self.create_client(JointMove, 'right_arm/joint_move')

        # Master gripper-control switch: R enables teleop+recording only, L also enables both grippers
        self.gripper_control_enabled = False
        # When L is pressed, use the actual gripper state (current_pos) as baseline without publishing set_pos
        self._last_left_gripper_pos = None
        self._last_right_gripper_pos = None
        # Debounce: publish only after N consecutive identical states to avoid threshold chatter
        self._debounce_count = 8
        # Temporarily block "close" commands after pressing L to avoid false RT=T spikes from VR input
        self._gripper_close_cooldown_sec = 0.12
        self._gripper_enabled_at = None
        self._lt_consecutive, self._lt_last_state, self._lt_last_published = 0, None, None
        self._rt_consecutive, self._rt_last_state, self._rt_last_published = 0, None, None

        self.create_subscription(JointState, '/left_gripper/current_pos', self._left_gripper_pos_cb, 10)
        self.create_subscription(JointState, '/right_gripper/current_pos', self._right_gripper_pos_cb, 10)

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
        self.gripper_control_enabled = False
        request = SetBool.Request()
        request.data = False
        future = self.service_clients["start_left_arm_servo_mode"].call_async(request)
        
        time.sleep(0.1)
        
        # for client in self.pre_service_clients.values():
        #     if client.service_is_ready():
        #         request = SetBool.Request()
        #         request.data = True
        #         future = client.call_async(request)

        # time.sleep(1)

        self.get_logger().info('Move to default pose.')
        request = JointMove.Request()
        request.robot_arm_index = -1 
        request.move_mode = 0 # absolute move
        request.is_blocking = False
        request.speed = 1.0
        # left arm value
        # request.joint_positions = [-1.359192608283104, -0.9613273519984766, 2.034198696491911, 
                                #   -1.481068949949868, 0.3909188458616899, -0.9122486934323962, 0.382808040271289,
                                #   -2.03533306109991, 1.05267788504786, 1.556727709524621, -1.5725765626319308,
                                #    -0.5133885994741321, 1.0164448497764575, 0.507399053106668]
        # building E demo horizon
        # request.joint_positions = [0.25576054858724906,-1.3941690064930703,-4.929490486040264,
        #                            -1.2750153784594176, -0.27841492227813547, 1.386524464369335,1.8050020591200155,
        #                            -3.470377778080485,1.3798224000416772,1.572506749461851,
        #                            -1.3929821826017144,-2.963062924403293,1.5340745993329359,1.728312291787385]
        
        # E demo vertical
#         request.joint_positions = [  15.2
# , 2.9
# , 26.8
# , -75.0
# , 127.7
# , 23.1
# , -195.5
# , -26.7
# , -8.345
# , -18.834
# , -75.199
# , -99.680
# , -32.756
# , -6.275]
        # Initial pose for the current robot setup
        # request.joint_positions = [-0.34603, 0.35585, 0.22068, -1.64561, 0.12012, -0.90419, 1.37207, 1.64884, -0.35538, -1.46538, -1.29316, -0.27703, 1.1691, 0.58005]
#         request.joint_positions = [math.radians(float(v)) for v in request.joint_positions]
        # Downward-facing pose
        # request.joint_positions = [0.0
        # , 0.0
        # , -30.0
        # , -90.0
        # , 0.0
        # , 0.0
        # , -90.0
        # , -0.0
        # , 0.0
        # , 30.0
        # , -90.0
        # , 0.0
        # , 0.0
        # , -90.0]  
        # request.joint_positions = [math.radians(float(v)) for v in request.joint_positions]  

        request.joint_positions = [  147.526
, 50.410
, -80.504
, -126.105
, -2.818
, 55.294
, 195.815
, -154.613
, 53.366
, 78.683
, -126.590
, 176.205
, 65.272
, -188.216]
        request.joint_positions = [math.radians(float(v)) for v in request.joint_positions]
                           
        future = self.dual_arm_joint_move_client.call_async(request)


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
        if msg.data == 'LG=T':
            if self.state != 'OPERATIONAL':
                self.get_logger().info('left trans to operational.')
                self.to_operational()
                self.get_logger().info('auto start recording.')
                self.call_service('start_recording')
            # Pressing L enables both grippers (LT/RT drive left/right grippers separately)
            # Use the actual gripper state as the baseline and keep current_pos unchanged
            self.gripper_control_enabled = True
            self._gripper_enabled_at = time.time()
            self._lt_consecutive, self._lt_last_state = 0, None
            self._rt_consecutive, self._rt_last_state = 0, None
            # Use current_pos as the published baseline: >0.5 means open(False), otherwise closed(True)
            # Keep None until current_pos arrives, then adopt the first VR state in the callback
            lp = self._last_left_gripper_pos
            rp = self._last_right_gripper_pos
            self._lt_last_published = (lp < 0.5) if lp is not None else None
            self._rt_last_published = (rp < 0.5) if rp is not None else None
            self.get_logger().info('gripper control enabled (both hands).')

    def right_side_button_callback(self, msg):
        if msg.data == 'RG=T' and self.state != 'OPERATIONAL':
            self.get_logger().info('right trans to operational.')
            self.to_operational()
            self.get_logger().info('auto start recording.')
            self.call_service('start_recording')
            # Pressing R enables teleop+recording only, without gripper control
            self.gripper_control_enabled = False

    def _left_gripper_pos_cb(self, msg):
        if msg.position:
            self._last_left_gripper_pos = msg.position[0]

    def _right_gripper_pos_cb(self, msg):
        if msg.position:
            self._last_right_gripper_pos = msg.position[0]

    def left_front_button_callback(self, msg):
        if self.state != 'OPERATIONAL' or not self.gripper_control_enabled:
            return
        current_lt = (msg.data == 'LT=T')
        if current_lt != self._lt_last_state:
            self._lt_last_state = current_lt
            self._lt_consecutive = 0
        self._lt_consecutive += 1
        if self._lt_consecutive < self._debounce_count:
            return
        if self._lt_last_published is None:
            self._lt_last_published = current_lt
            return
        if self._lt_last_published == current_lt:
            return
        # During the cooldown after L, block close commands to avoid false LT=T transitions
        if msg.data == 'LT=T' and self._gripper_enabled_at is not None:
            if (time.time() - self._gripper_enabled_at) < self._gripper_close_cooldown_sec:
                return
        self._lt_last_published = current_lt
        current_time = self.get_clock().now().to_msg()
        msg_gripper = JointState()
        msg_gripper.header.stamp = current_time
        if msg.data == 'LT=T':
            self.publisher_left_command_gripper_vel.publish(Int32(data=self.gripper_vel_close))
            msg_gripper.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_left_command_gripper_val.publish(msg_gripper)
            self.get_logger().info('invoke left gripper close')
        else:
            self.publisher_left_command_gripper_vel.publish(Int32(data=self.gripper_vel_open))
            msg_gripper.position = [0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_left_command_gripper_val.publish(msg_gripper)
            self.get_logger().info('invoke left gripper open.')

    def right_front_button_callback(self, msg):
        if self.state != 'OPERATIONAL' or not self.gripper_control_enabled:
            return
        current_rt = (msg.data == 'RT=T')
        if current_rt != self._rt_last_state:
            self._rt_last_state = current_rt
            self._rt_consecutive = 0
        self._rt_consecutive += 1
        if self._rt_consecutive < self._debounce_count:
            return
        if self._rt_last_published is None:
            self._rt_last_published = current_rt
            return
        if self._rt_last_published == current_rt:
            return
        # During the cooldown after L, block close commands to avoid false RT=T transitions
        if msg.data == 'RT=T' and self._gripper_enabled_at is not None:
            if (time.time() - self._gripper_enabled_at) < self._gripper_close_cooldown_sec:
                return
        self._rt_last_published = current_rt
        current_time = self.get_clock().now().to_msg()
        msg_gripper = JointState()
        msg_gripper.header.stamp = current_time
        if msg.data == 'RT=T':
            self.publisher_right_command_gripper_vel.publish(Int32(data=self.gripper_vel_close))
            msg_gripper.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_right_command_gripper_val.publish(msg_gripper)
            self.get_logger().info('invoke right gripper close')
        else:
            self.publisher_right_command_gripper_vel.publish(Int32(data=self.gripper_vel_open))
            msg_gripper.position = [0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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
