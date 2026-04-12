#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class LeftGripperPositionPublisher(Node):
    def __init__(self):
        super().__init__('left_gripper_position_publisher')
        self.publisher_ = self.create_publisher(JointState, '/left_gripper/set_pos', 10)
        
        # Configure publishing parameters
        self.publish_frequency = 90.0  # Hz
        self.timer = self.create_timer(1.0/self.publish_frequency, self.timer_callback)
        
        # Initialize joint state message
        self.joint_state = JointState()
        self.joint_state.name = ['left_gripper_joint']  # Replace with your actual joint name
        self.joint_state.position = [0.0]  # Initial position
        
        # Configuration for position changes
        self.position_range = [0.0, 1.0]  # Min and max position values
        self.position_step = 0.05  # How much to change position each step
        self.current_direction = 1  # 1 for increasing, -1 for decreasing
        
        self.get_logger().info(f"Publishing to /left_gripper/set_pos at {self.publish_frequency} Hz")

    def timer_callback(self):
        # Update position (example: oscillating between min and max)
        new_position = self.joint_state.position[0] + (self.position_step * self.current_direction)
        
        # Reverse direction if we hit limits
        if new_position >= self.position_range[1]:
            new_position = self.position_range[1]
            self.current_direction = -1
        elif new_position <= self.position_range[0]:
            new_position = self.position_range[0]
            self.current_direction = 1
            
        self.joint_state.position = [new_position]
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f"Publishing position: {new_position}")

def main(args=None):
    rclpy.init(args=args)
    publisher = LeftGripperPositionPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info("Shutting down publisher...")
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()