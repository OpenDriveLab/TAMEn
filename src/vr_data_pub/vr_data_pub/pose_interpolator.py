#!/usr/bin/env python3
"""
Pose interpolation node.
Receives VR pose data around 90 Hz and publishes interpolated poses
to the robot driver at a stable 1000 Hz.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from collections import deque
from threading import Lock
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class PoseInterpolator(Node):
    """
    Pose interpolation node.
    Subscribes to robot_command_servo_p from vr_robot_pose_converter
    and republishes interpolated poses to interpolated_robot_command at 1000 Hz.
    """

    def __init__(self):
        super().__init__('pose_interpolator')
        
        # Parameters
        self.declare_parameter('output_frequency', 1000.0)  # Target output frequency: 1000 Hz
        self.declare_parameter('buffer_size', 10)  # Larger buffer to absorb input-rate jitter
        self.declare_parameter('max_extrapolation_time', 0.05)  # Maximum extrapolation window: 50 ms
        
        self.output_freq = self.get_parameter('output_frequency').get_parameter_value().double_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.max_extrapolation_time = self.get_parameter('max_extrapolation_time').get_parameter_value().double_value
        
        # Pose buffer
        self.pose_buffer = deque(maxlen=self.buffer_size)
        self.buffer_lock = Lock()
        
        # Last published pose
        self.last_pose = None
        self.last_timestamp = None
        
        # Most recently received samples used for extrapolation
        self.last_received_pose = None
        self.last_received_timestamp = None
        self.second_last_received_pose = None
        self.second_last_received_timestamp = None
        
        # Statistics
        self.publish_count = 0
        self.interpolation_count = 0
        self.extrapolation_count = 0
        self.input_msg_count = 0
        
        # Frequency monitoring
        self.last_stat_time = self.get_clock().now()
        self.last_stat_publish_count = 0
        
        # Subscribe to poses from the converter
        self.pose_sub = self.create_subscription(
            JointState,
            'robot_command_servo_p',
            self.pose_callback,
            10
        )
        
        # Publish interpolated poses
        self.interpolated_pub = self.create_publisher(
            JointState,
            'interpolated_robot_command',
            10
        )
        
        # Fixed-rate publish timer
        timer_period = 1.0 / self.output_freq  # 1ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Statistics timer (print every 5 seconds)
        self.stats_timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info(f'Pose interpolator started - output frequency: {self.output_freq} Hz')
        self.get_logger().info('Position interpolation: linear | Orientation interpolation: SLERP')

    def pose_callback(self, msg):
        """Receive VR pose data (60-90 Hz, with input-rate variation)."""
        with self.buffer_lock:
            current_time = self.get_clock().now()
            pose = list(msg.position)
            
            # Update the most recent samples used for extrapolation
            if self.last_received_pose is not None:
                self.second_last_received_pose = self.last_received_pose
                self.second_last_received_timestamp = self.last_received_timestamp
            
            self.last_received_pose = pose
            self.last_received_timestamp = current_time
            
            # Push into the buffer
            self.pose_buffer.append({
                'pose': pose,
                'timestamp': current_time
            })
            
            self.input_msg_count += 1
            # self.get_logger().debug(f'Received pose data, buffer size: {len(self.pose_buffer)}')

    def extrapolate_pose(self, pose1, pose2, time1, time2, target_time):
        """
        Extrapolate a pose when the target time is newer than the latest sample.

        The extrapolation follows the recent motion trend inferred from the last
        two poses.

        Args:
            pose1: Earlier pose sample.
            pose2: Later pose sample.
            time1: Timestamp of pose1.
            time2: Timestamp of pose2.
            target_time: Target timestamp.
        """
        # Compute time intervals
        dt_data = (time2 - time1).nanoseconds / 1e9
        dt_extrapolate = (target_time - time2).nanoseconds / 1e9
        
        if dt_data <= 0:
            return pose2
        
        # Clamp extrapolation duration to avoid excessive extrapolation
        if dt_extrapolate > self.max_extrapolation_time:
            dt_extrapolate = self.max_extrapolation_time
        
        # Compute extrapolation factor
        alpha = 1.0 + (dt_extrapolate / dt_data)
        
        # Split translation and orientation
        pos1 = np.array(pose1[:3])
        pos2 = np.array(pose2[:3])
        rpy1 = np.array(pose1[3:])
        rpy2 = np.array(pose2[3:])
        
        # Position extrapolation: linear
        velocity_pos = (pos2 - pos1) / dt_data
        extrapolated_pos = pos2 + velocity_pos * dt_extrapolate
        
        # Orientation extrapolation: use SLERP with alpha > 1.0
        try:
            rot1 = R.from_euler('xyz', rpy1, degrees=False)
            rot2 = R.from_euler('xyz', rpy2, degrees=False)
            
            key_times = [0, 1]
            key_rots = R.from_quat([rot1.as_quat(), rot2.as_quat()])
            slerp = Slerp(key_times, key_rots)
            
            # Extrapolate with alpha > 1.0
            extrapolated_rot = slerp([alpha])[0]
            extrapolated_rpy = extrapolated_rot.as_euler('xyz', degrees=False)
            
        except Exception as e:
            # Fall back to simple extrapolation
            angular_velocity = (rpy2 - rpy1) / dt_data
            extrapolated_rpy = rpy2 + angular_velocity * dt_extrapolate
        
        extrapolated_pose = np.concatenate([extrapolated_pos, extrapolated_rpy])
        return extrapolated_pose.tolist()
    
    def interpolate_pose(self, pose1, pose2, alpha):
        """
        Interpolate between two poses.

        pose format: [x, y, z, roll, pitch, yaw]
        The first 3 values are translation in mm and use linear interpolation.
        The last 3 values are orientation in radians and use SLERP.

        alpha: interpolation factor in [0.0, 1.0].
        """
        # Split translation and orientation
        pos1 = np.array(pose1[:3])  # Translation (x, y, z) in mm
        pos2 = np.array(pose2[:3])
        
        rpy1 = np.array(pose1[3:])  # Orientation (roll, pitch, yaw) in radians
        rpy2 = np.array(pose2[3:])
        
        # Position: simple linear interpolation
        interpolated_pos = pos1 * (1.0 - alpha) + pos2 * alpha
        
        # Orientation: spherical linear interpolation (SLERP)
        try:
            # Convert Euler angles to rotation objects
            rot1 = R.from_euler('xyz', rpy1, degrees=False)
            rot2 = R.from_euler('xyz', rpy2, degrees=False)
            
            # Perform spherical linear interpolation with SLERP
            key_times = [0, 1]
            key_rots = R.from_quat([rot1.as_quat(), rot2.as_quat()])
            slerp = Slerp(key_times, key_rots)
            
            # Interpolate the intermediate rotation
            interpolated_rot = slerp([alpha])[0]
            
            # Convert back to Euler angles
            interpolated_rpy = interpolated_rot.as_euler('xyz', degrees=False)
            
        except Exception as e:
            # Fall back to linear interpolation if SLERP fails
            self.get_logger().warn(f'SLERP failed, falling back to linear interpolation: {e}')
            interpolated_rpy = rpy1 * (1.0 - alpha) + rpy2 * alpha
        
        # Merge translation and orientation
        interpolated_pose = np.concatenate([interpolated_pos, interpolated_rpy])
        return interpolated_pose.tolist()

    def timer_callback(self):
        """
        Publish interpolated or extrapolated poses at a strict 1000 Hz.

        This keeps the output stable even if the input rate fluctuates between
        roughly 60 and 90 Hz.
        """
        with self.buffer_lock:
            current_time = self.get_clock().now()
            
            # Wait until at least one sample has been received
            if self.last_received_pose is None:
                return
            
            # If only one sample is available, publish it directly
            if len(self.pose_buffer) < 2:
                if self.last_pose is None:
                    self.last_pose = self.last_received_pose
                self.publish_pose(self.last_pose)
                return
            
            # Get the latest two poses
            pose_data_1 = self.pose_buffer[-2]
            pose_data_2 = self.pose_buffer[-1]
            
            time1 = pose_data_1['timestamp']
            time2 = pose_data_2['timestamp']
            
            # Compute the time interval
            time_diff = (time2 - time1).nanoseconds / 1e9
            
            if time_diff <= 0:
                # Invalid timestamps, fall back to the latest pose
                output_pose = pose_data_2['pose']
            else:
                # Compute the current time position relative to the sampled data
                time_since_pose1 = (current_time - time1).nanoseconds / 1e9
                time_since_pose2 = (current_time - time2).nanoseconds / 1e9
                
                if time_since_pose2 <= 0:
                    # Current time lies between two samples, so interpolate
                    alpha = time_since_pose1 / time_diff
                    alpha = min(max(alpha, 0.0), 1.0)
                    
                    output_pose = self.interpolate_pose(
                        pose_data_1['pose'],
                        pose_data_2['pose'],
                        alpha
                    )
                    self.interpolation_count += 1
                    
                else:
                    # Current time is newer than the latest sample, so extrapolate
                    # Use the two most recently received samples for extrapolation
                    if (self.second_last_received_pose is not None and 
                        self.second_last_received_timestamp is not None):
                        output_pose = self.extrapolate_pose(
                            self.second_last_received_pose,
                            self.last_received_pose,
                            self.second_last_received_timestamp,
                            self.last_received_timestamp,
                            current_time
                        )
                        self.extrapolation_count += 1
                    else:
                        # Not enough data for extrapolation, use the latest pose
                        output_pose = self.last_received_pose
            
            # Publish the pose while maintaining the 1000 Hz output rate
            self.publish_pose(output_pose)
            
            # Update the last published pose
            self.last_pose = output_pose
            self.last_timestamp = current_time

    def publish_pose(self, pose):
        """Publish a pose."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = pose
        self.interpolated_pub.publish(msg)
        self.publish_count += 1
    
    def print_statistics(self):
        """Print detailed statistics, including the actual output frequency."""
        current_time = self.get_clock().now()
        time_elapsed = (current_time - self.last_stat_time).nanoseconds / 1e9
        
        if time_elapsed > 0:
            # Compute the actual output frequency
            publish_count_diff = self.publish_count - self.last_stat_publish_count
            actual_output_freq = publish_count_diff / time_elapsed
            
            # Compute the average input frequency over the last 5 seconds
            input_freq = self.input_msg_count / time_elapsed
            
            self.get_logger().info(
                f'=== Interpolator Statistics ===\n'
                f'  Output frequency: {actual_output_freq:.1f} Hz (target: {self.output_freq:.0f} Hz)\n'
                f'  Input frequency: {input_freq:.1f} Hz (VR data)\n'
                f'  Total published: {self.publish_count}\n'
                f'  Interpolations: {self.interpolation_count}\n'
                f'  Extrapolations: {self.extrapolation_count}\n'
                f'  Buffer size: {len(self.pose_buffer)}/{self.buffer_size}'
            )
            
            # Reset statistics counters
            self.last_stat_time = current_time
            self.last_stat_publish_count = self.publish_count
            self.input_msg_count = 0


def main(args=None):
    rclpy.init(args=args)
    
    pose_interpolator = PoseInterpolator()
    
    try:
        rclpy.spin(pose_interpolator)
    except KeyboardInterrupt:
        pass
    finally:
        pose_interpolator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

