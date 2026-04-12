import rclpy
from rclpy.node import Node
from pyDHgripper.AG95.Gripper import Gripper as AG95Base
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
#from k1_msgs.srv import JointMove, LinearMove, KineInverse7
#from k1_robot import jkrc
import math
import numpy as np
import threading
import serial
import crcmod

import time

class AG95NoInit(AG95Base):
    """AG95 gripper wrapper that skips the init motion on startup."""
    def __init__(self, port="/dev/ttyUSB0") -> None:
        self.ser = serial.Serial(port=port, baudrate=115200)
        self.crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
        # Skip init_state() and init_feedback() to avoid open/close motion at startup
class GripperController:
    def __init__(self, gripper, name, node):
        self.gripper = gripper
        self.name = name
        self.node = node  # Pass the ROS 2 node for logging and clock access
        self.pos_publisher = None
        self.force_subscription = None
        self.pos_subscription = None
        self.vel_subscription = None
        # self.pre_pos = None
        self.last_pos= 0
        # Read the current position as the initial state to avoid startup motion
        self.pos = self.gripper.read_pos()
        # self.pos_lock = threading.Lock()
        
        # # Initialize the position polling thread
        # self._running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
    def _update_loop(self):
        """Background loop that continuously polls the current position."""
        while True:
            current_pos = self.gripper.read_pos()
            # Retry once if read_pos spuriously returns 0 while the gripper is not closed
            if current_pos == 0 and self.last_pos > 300:
                time.sleep(0.02)
                retry = self.gripper.read_pos()
                if retry > 0:
                    current_pos = retry
            self.gripper.set_pos(int(self.pos))
            self.last_pos = current_pos
            time.sleep(0.1)
    def setup_publishers(self):
        self.pos_publisher = self.node.create_publisher(JointState, f'{self.name}/current_pos', 10)

    def setup_subscriptions(self):
        self.force_subscription = self.node.create_subscription(
            Int32, f'{self.name}/set_force', self.force_callback, 10
        )
        self.pos_subscription = self.node.create_subscription(
            JointState, f'{self.name}/set_pos', self.pos_callback, 10
        )
        self.vel_subscription = self.node.create_subscription(
            Int32, f'{self.name}/set_vel', self.vel_callback, 10
        )

    def force_callback(self, msg):
        self.gripper.set_force(val=msg.data)
        self.node.get_logger().info(f'{self.name} force set to: {msg.data}')

    def pos_callback(self, msg):
        current_time = self.node.get_clock().now().to_msg()  # Use the node's clock
        msg.header.stamp = current_time
        self.pos = msg.position[0] * 1000  # Use the first value in the position array
        
        # self.gripper.set_pos(val=int(pos))
        # if self.pre_pos != None:
        #     diff_pos = round(abs(self.pre_pos - pos))
        #     if diff_pos != 0:
        #         self.gripper.set_pos(val=int(pos))
        #         self.node.get_logger().info(f'{self.name} position set to: {pos}')
        # self.pre_pos = pos

    def vel_callback(self, msg):
        self.gripper.set_vel(val=msg.data)
        self.node.get_logger().info(f'{self.name} velocity set to: {msg.data}')

    def publish_position(self):
      
        current_time = self.node.get_clock().now().to_msg()  # Use the node's clock
        current_pos = self.last_pos
        # print(current_pos)
        msg = JointState()
        msg.header.stamp = current_time
        msg.position = [current_pos * 0.001] # Ensure position is a list
        self.pos_publisher.publish(msg)
        # self.node.get_logger().info(f'{self.name} current position: {current_pos}')


class DHGripperNode(Node):
    def __init__(self):
        super().__init__('dhgripper_node')

        # Initialize grippers (use AG95NoInit to skip the startup motion)
        self.grippers = {
            'left_gripper': GripperController(AG95NoInit(port='/dev/ttyUSB0'), 'left_gripper', self),
            'right_gripper': GripperController(AG95NoInit(port='/dev/ttyUSB1'), 'right_gripper', self),
        }
        for gripper in self.grippers.values():
            gripper.setup_publishers()
            gripper.setup_subscriptions()

        # Timer to publish current positions
        self.timer = self.create_timer(0.001, self.timer_callback) 

    def timer_callback(self):
        for gripper in self.grippers.values():
            gripper.publish_position()
        # threads = []
        # for gripper in self.grippers.values():
        #     thread = threading.Thread(
        #             target=gripper.publish_position,
        #             daemon=True  # Thread exits when main program does
        #     )
        #     threads.append(thread)
        #     thread.start()

        #     # Wait for all threads to complete (optional)
        # for thread in threads:
        #     thread.join()


def main(args=None):
    rclpy.init(args=args)
    node = DHGripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
