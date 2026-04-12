#!/usr/bin/env python3
"""
Topic Data Recorder
Record /robot_command_servo_p, /robot_command_servo_p_filtered,
/left_arm/vr_pose, and EDG state data into CSV files.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import csv
import os
from datetime import datetime
import json
import threading
import re
import numpy as np
from scipy.spatial.transform import Rotation as R


class TopicDataRecorder(Node):
    def __init__(self):
        super().__init__('topic_data_recorder')
        
        # Declare parameters
        self.declare_parameter('output_dir', '/root/K1-W/data')
        self.declare_parameter('file_prefix', 'data')
        self.declare_parameter('save_format', 'csv')  # csv or json
        # Whether to skip edg_stat logging when /robot_command_servo_p has no valid values
        self.declare_parameter('skip_edg_stat_without_valid_servo_p', True)
        # Whether to skip /robot_command_servo_p_filtered logging when /robot_command_servo_p has no valid values
        # This avoids log growth when raw input stops but filtered interpolation is still being published
        self.declare_parameter('skip_servo_p_filtered_without_valid_servo_p', True)
        # A servo_p sample is considered invalid if the latest valid one is older than this threshold
        self.declare_parameter('valid_servo_p_timeout_sec', 0.5)
        
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        file_prefix = self.get_parameter('file_prefix').get_parameter_value().string_value
        self.save_format = self.get_parameter('save_format').get_parameter_value().string_value
        self.skip_edg_stat_without_valid_servo_p = (
            self.get_parameter('skip_edg_stat_without_valid_servo_p').get_parameter_value().bool_value
        )
        self.skip_servo_p_filtered_without_valid_servo_p = (
            self.get_parameter('skip_servo_p_filtered_without_valid_servo_p').get_parameter_value().bool_value
        )
        self.valid_servo_p_timeout_sec = (
            self.get_parameter('valid_servo_p_timeout_sec').get_parameter_value().double_value
        )
        
        # Expand the ~ path
        output_dir = os.path.expanduser(output_dir)
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate timestamped filenames
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        if self.save_format == 'csv':
            self.servo_p_file = os.path.join(output_dir, f'{file_prefix}_servo_p_{timestamp}.csv')
            self.servo_p_filtered_file = os.path.join(output_dir, f'{file_prefix}_servo_p_filtered_{timestamp}.csv')
            self.vr_pose_file = os.path.join(output_dir, f'{file_prefix}_vr_pose_{timestamp}.csv')
            self.edg_stat_file = os.path.join(output_dir, f'{file_prefix}_edg_stat_{timestamp}.csv')
            
            # Initialize CSV files
            self.servo_p_writer = None
            self.servo_p_filtered_writer = None
            self.vr_pose_writer = None
            self.edg_stat_writer = None
            self.servo_p_file_handle = None
            self.servo_p_filtered_file_handle = None
            self.vr_pose_file_handle = None
            self.edg_stat_file_handle = None
            
            self._init_csv_files()
        else:  # json
            self.servo_p_file = os.path.join(output_dir, f'{file_prefix}_servo_p_{timestamp}.json')
            self.servo_p_filtered_file = os.path.join(output_dir, f'{file_prefix}_servo_p_filtered_{timestamp}.json')
            self.vr_pose_file = os.path.join(output_dir, f'{file_prefix}_vr_pose_{timestamp}.json')
            self.edg_stat_file = os.path.join(output_dir, f'{file_prefix}_edg_stat_{timestamp}.json')
            self.servo_p_data = []
            self.servo_p_filtered_data = []
            self.vr_pose_data = []
            self.edg_stat_data = []
        
        self.get_logger().info(f'Recording data to: {output_dir}')
        self.get_logger().info(f'Servo P file: {self.servo_p_file}')
        self.get_logger().info(f'Servo P (filtered) file: {self.servo_p_filtered_file}')
        self.get_logger().info(f'VR Pose file: {self.vr_pose_file}')
        self.get_logger().info(f'EDG Stat file: {self.edg_stat_file}')
        
        # Create subscribers
        self.servo_p_subscription = self.create_subscription(
            JointState,
            '/robot_command_servo_p',
            self.servo_p_callback,
            10
        )

        self.servo_p_filtered_subscription = self.create_subscription(
            JointState,
            '/robot_command_servo_p_filtered',
            self.servo_p_filtered_callback,
            10
        )
        
        self.left_vr_pose_subscription = self.create_subscription(
            String,
            '/left_arm/vr_pose',
            lambda msg: self.vr_pose_callback(msg, 'left'),
            10
        )
        
        self.right_vr_pose_subscription = self.create_subscription(
            String,
            '/right_arm/vr_pose',
            lambda msg: self.vr_pose_callback(msg, 'right'),
            10
        )
        
        # EDG state-data subscriptions
        self.left_arm_joint_state_sub = self.create_subscription(
            JointState, 'left_arm/joint_states', 
            lambda msg: self.edg_joint_state_callback(msg, 'left'), 10)
        self.right_arm_joint_state_sub = self.create_subscription(
            JointState, 'right_arm/joint_states', 
            lambda msg: self.edg_joint_state_callback(msg, 'right'), 10)
        self.left_arm_tcp_pose_sub = self.create_subscription(
            JointState, 'left_arm/cur_tcp_pose', 
            lambda msg: self.edg_tcp_pose_callback(msg, 'left'), 10)
        self.right_arm_tcp_pose_sub = self.create_subscription(
            JointState, 'right_arm/cur_tcp_pose', 
            lambda msg: self.edg_tcp_pose_callback(msg, 'right'), 10)
        self.left_arm_raw_torque_sub = self.create_subscription(
            JointState, 'left_arm/raw_torque_sensor_val', 
            lambda msg: self.edg_torque_callback(msg, 'left'), 10)
        self.right_arm_raw_torque_sub = self.create_subscription(
            JointState, 'right_arm/raw_torque_sensor_val', 
            lambda msg: self.edg_torque_callback(msg, 'right'), 10)
        
        self.get_logger().info('Topic Data Recorder started')
        self.get_logger().info('Subscribed to: /robot_command_servo_p')
        self.get_logger().info('Subscribed to: /robot_command_servo_p_filtered')
        self.get_logger().info('Subscribed to: /left_arm/vr_pose')
        self.get_logger().info('Subscribed to: /right_arm/vr_pose')
        self.get_logger().info('Subscribed to: left_arm/joint_states, right_arm/joint_states')
        self.get_logger().info('Subscribed to: left_arm/cur_tcp_pose, right_arm/cur_tcp_pose')
        self.get_logger().info('Subscribed to: left_arm/raw_torque_sensor_val, right_arm/raw_torque_sensor_val')
        
        # Statistics
        self.servo_p_count = 0
        self.servo_p_filtered_count = 0
        self.vr_pose_count = 0
        self.left_vr_pose_count = 0
        self.right_vr_pose_count = 0
        self.edg_stat_count = 0
        
        # Cache left/right vr_pose data so they can be written together
        self.left_vr_pose_cache = None
        self.right_vr_pose_cache = None
        self.left_vr_pose_timestamp = None  # Stored as a floating-point timestamp
        self.right_vr_pose_timestamp = None  # Stored as a floating-point timestamp
        
        # EDG state-data cache
        self.left_arm_joint_state = None
        self.right_arm_joint_state = None
        self.left_arm_tcp_pose = None
        self.right_arm_tcp_pose = None
        self.left_arm_raw_torque = None
        self.right_arm_raw_torque = None
        self.edg_data_lock = threading.Lock()
        self.last_edg_collect_timestamp = 0.0
        self.min_edg_collect_interval = 0.01  # Minimum collection interval: 10 ms

        # servo_p validity state used to gate edg_stat logging
        self._servo_p_state_lock = threading.Lock()
        self._last_valid_servo_p_rx_time = None  # float seconds (node clock)
        self._last_servo_p_was_valid = False
        
        # Create a timer to print statistics periodically
        self.create_timer(5.0, self.print_stats)
    
    def _init_csv_files(self):
        """Initialize CSV files."""
        # Servo P CSV
        self.servo_p_file_handle = open(self.servo_p_file, 'w', newline='')
        self.servo_p_writer = csv.writer(self.servo_p_file_handle)
        # Write headers
        self.servo_p_writer.writerow([
            'timestamp',
            'position_0', 'position_1', 'position_2', 'position_3', 
            'position_4', 'position_5', 'position_6', 'position_7',
            'position_8', 'position_9', 'position_10', 'position_11',
            'velocity', 'effort'
        ])

        # Servo P (filtered) CSV
        self.servo_p_filtered_file_handle = open(self.servo_p_filtered_file, 'w', newline='')
        self.servo_p_filtered_writer = csv.writer(self.servo_p_filtered_file_handle)
        self.servo_p_filtered_writer.writerow([
            'timestamp',
            'position_0', 'position_1', 'position_2', 'position_3',
            'position_4', 'position_5', 'position_6', 'position_7',
            'position_8', 'position_9', 'position_10', 'position_11',
            'velocity', 'effort'
        ])
        
        # VR Pose CSV
        self.vr_pose_file_handle = open(self.vr_pose_file, 'w', newline='')
        self.vr_pose_writer = csv.writer(self.vr_pose_file_handle)
        # Write headers in a servo_p-like format with 12 values and tcp0/tcp1 prefixes
        self.vr_pose_writer.writerow([
            'timestamp',
            'tcp0_x', 'tcp0_y', 'tcp0_z', 'tcp0_rx', 'tcp0_ry', 'tcp0_rz',
            'tcp1_x', 'tcp1_y', 'tcp1_z', 'tcp1_rx', 'tcp1_ry', 'tcp1_rz'
        ])
        
        # EDG Stat CSV
        self.edg_stat_file_handle = open(self.edg_stat_file, 'w', newline='')
        self.edg_stat_writer = csv.writer(self.edg_stat_file_handle)
        # Write headers
        fieldnames = ['timestamp']
        fieldnames.extend([f'left_joint_{i}' for i in range(7)])
        fieldnames.extend([f'right_joint_{i}' for i in range(7)])
        fieldnames.extend(['left_tcp_x', 'left_tcp_y', 'left_tcp_z', 
                          'left_tcp_rx', 'left_tcp_ry', 'left_tcp_rz'])
        fieldnames.extend(['right_tcp_x', 'right_tcp_y', 'right_tcp_z',
                          'right_tcp_rx', 'right_tcp_ry', 'right_tcp_rz'])
        fieldnames.extend([f'left_torque_{i}' for i in range(6)])
        fieldnames.extend([f'right_torque_{i}' for i in range(6)])
        self.edg_stat_writer.writerow(fieldnames)
    
    def servo_p_callback(self, msg):
        """Handle /robot_command_servo_p messages."""
        try:
            # Update servo_p validity: at least 12 dimensions and all values must be finite
            now = self.get_clock().now()
            now_sec = now.nanoseconds / 1e9
            is_valid = False
            if len(msg.position) >= 12:
                try:
                    first12 = np.asarray(msg.position[:12], dtype=float)
                    is_valid = bool(np.all(np.isfinite(first12)))
                except Exception:
                    is_valid = False
            with self._servo_p_state_lock:
                self._last_servo_p_was_valid = is_valid
                if is_valid:
                    self._last_valid_servo_p_rx_time = now_sec

            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            # Convert to a floating-point timestamp with 0.01 s precision
            timestamp_float = timestamp_sec + round(timestamp_nanosec / 1_000_000_000, 2)
            
            if self.save_format == 'csv':
                # Build the output row
                row = [
                    timestamp_float
                ]
                
                # Append position values (up to 12; pad with empty strings if fewer)
                for i in range(12):
                    if i < len(msg.position):
                        row.append(msg.position[i])
                    else:
                        row.append('')
                
                # Append velocity and effort if present
                if len(msg.velocity) > 0:
                    row.append(str(msg.velocity))
                else:
                    row.append('')
                
                if len(msg.effort) > 0:
                    row.append(str(msg.effort))
                else:
                    row.append('')
                
                self.servo_p_writer.writerow(row)
                self.servo_p_file_handle.flush()  # Flush immediately to disk
            else:  # json
                data = {
                    'timestamp': timestamp_float,
                    'position': list(msg.position),
                    'velocity': list(msg.velocity) if len(msg.velocity) > 0 else [],
                    'effort': list(msg.effort) if len(msg.effort) > 0 else []
                }
                self.servo_p_data.append(data)
            
            self.servo_p_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing servo_p message: {e}')

    def servo_p_filtered_callback(self, msg):
        """Handle /robot_command_servo_p_filtered messages (filtered/interpolated driver commands)."""
        try:
            # If gating is enabled, skip filtered logging when no recent valid servo_p has been received
            if self.skip_servo_p_filtered_without_valid_servo_p:
                now = self.get_clock().now()
                now_sec = now.nanoseconds / 1e9
                with self._servo_p_state_lock:
                    last_valid = self._last_valid_servo_p_rx_time
                if last_valid is None or (now_sec - last_valid) > float(self.valid_servo_p_timeout_sec):
                    return

            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            timestamp_float = timestamp_sec + round(timestamp_nanosec / 1_000_000_000, 2)

            if self.save_format == 'csv':
                row = [timestamp_float]
                for i in range(12):
                    if i < len(msg.position):
                        row.append(msg.position[i])
                    else:
                        row.append('')
                if len(msg.velocity) > 0:
                    row.append(str(msg.velocity))
                else:
                    row.append('')
                if len(msg.effort) > 0:
                    row.append(str(msg.effort))
                else:
                    row.append('')
                self.servo_p_filtered_writer.writerow(row)
                self.servo_p_filtered_file_handle.flush()
            else:  # json
                data = {
                    'timestamp': timestamp_float,
                    'position': list(msg.position),
                    'velocity': list(msg.velocity) if len(msg.velocity) > 0 else [],
                    'effort': list(msg.effort) if len(msg.effort) > 0 else []
                }
                self.servo_p_filtered_data.append(data)

            self.servo_p_filtered_count += 1

        except Exception as e:
            self.get_logger().error(f'Error processing servo_p_filtered message: {e}')
    
    def vr_pose_callback(self, msg, arm_type):
        """Handle /left_arm/vr_pose and /right_arm/vr_pose messages."""
        try:
            timestamp = self.get_clock().now()
            timestamp_sec = timestamp.nanoseconds // 1_000_000_000
            timestamp_nanosec = timestamp.nanoseconds % 1_000_000_000
            # Convert to a floating-point timestamp with 0.01 s precision
            timestamp_float = timestamp_sec + round(timestamp_nanosec / 1_000_000_000, 2)
            
            # Parse the string payload into a numeric list
            try:
                # Extract position data: LPos(...) or RPos(...)
                pos_match = re.search(r'[LR]Pos:\s*\(([^)]+)\)', msg.data)
                # Extract rotation data: LRot(...) or RRot(...)
                rot_match = re.search(r'[LR]Rot:\s*\(([^)]+)\)', msg.data)
                
                if pos_match and rot_match:
                    # Parse position values
                    pos_str = pos_match.group(1)
                    pos_values = [float(x.strip()) for x in pos_str.split(',')]
                    # Parse rotation values
                    rot_str = rot_match.group(1)
                    rot_values = [float(x.strip()) for x in rot_str.split(',')]
                    # Merge into 6 values: x, y, z, rx, ry, rz
                    pose_values = pos_values + rot_values
                else:
                    # Fall back to direct number parsing for backward compatibility
                    pose_values = [float(x) for x in re.findall(r'-?\d+\.?\d*', msg.data)]
                    if len(pose_values) >= 6:
                        pose_values = pose_values[:6]
                    else:
                        raise ValueError(f'Could not extract 6 values from: {msg.data}')
            except (ValueError, AttributeError) as e:
                self.get_logger().warn(f'Failed to parse vr_pose data: {msg.data}, error: {e}')
                return
            
            if len(pose_values) != 6:
                self.get_logger().warn(f'Expected 6 values in vr_pose, got {len(pose_values)}: {msg.data}')
                return
            
            # Cache the data
            if arm_type == 'left':
                self.left_vr_pose_cache = pose_values
                self.left_vr_pose_timestamp = timestamp_float
                self.left_vr_pose_count += 1
            else:  # right
                self.right_vr_pose_cache = pose_values
                self.right_vr_pose_timestamp = timestamp_float
                self.right_vr_pose_count += 1
            
            # Write a row once both left and right arm data are ready
            if self.left_vr_pose_cache is not None and self.right_vr_pose_cache is not None:
                # Use the newer timestamp
                if self.left_vr_pose_timestamp > self.right_vr_pose_timestamp:
                    ts_float = self.left_vr_pose_timestamp
                else:
                    ts_float = self.right_vr_pose_timestamp
                
                if self.save_format == 'csv':
                    # Format: timestamp, tcp0(6 values), tcp1(6 values)
                    row = [
                        ts_float
                    ]
                    # Append left-arm data (tcp0)
                    row.extend(self.left_vr_pose_cache)
                    # Append right-arm data (tcp1)
                    row.extend(self.right_vr_pose_cache)
                    
                    self.vr_pose_writer.writerow(row)
                    self.vr_pose_file_handle.flush()  # Flush immediately to disk
                    self.vr_pose_count += 1
                else:  # json
                    data = {
                        'timestamp': ts_float,
                        'tcp0': self.left_vr_pose_cache,
                        'tcp1': self.right_vr_pose_cache
                    }
                    self.vr_pose_data.append(data)
                    self.vr_pose_count += 1

                # Clear caches after a successful write
                self.left_vr_pose_cache = None
                self.right_vr_pose_cache = None
            
        except Exception as e:
            self.get_logger().error(f'Error processing vr_pose message: {e}')
    
    def edg_joint_state_callback(self, msg, arm_type):
        """Handle left/right arm joint-state callbacks."""
        with self.edg_data_lock:
            if arm_type == 'left':
                self.left_arm_joint_state = msg
            else:
                self.right_arm_joint_state = msg
            self._collect_edg_data()

    def edg_tcp_pose_callback(self, msg, arm_type):
        """Handle left/right arm TCP-pose callbacks."""
        with self.edg_data_lock:
            if arm_type == 'left':
                self.left_arm_tcp_pose = msg
            else:
                self.right_arm_tcp_pose = msg

    def edg_torque_callback(self, msg, arm_type):
        """Handle left/right arm torque-sensor callbacks."""
        with self.edg_data_lock:
            if arm_type == 'left':
                self.left_arm_raw_torque = msg
            else:
                self.right_arm_raw_torque = msg

    def _collect_edg_data(self):
        """Collect and write EDG state data."""
        # Check whether enough data is available
        if (self.left_arm_joint_state is None or self.right_arm_joint_state is None or
            self.left_arm_tcp_pose is None or self.right_arm_tcp_pose is None):
            return
        
        # Get the current timestamp
        clock_time = self.get_clock().now()
        timestamp = clock_time.seconds_nanoseconds()[0] + clock_time.seconds_nanoseconds()[1] / 1e9

        # If gating is enabled, skip edg_stat logging when no recent valid servo_p exists
        if self.skip_edg_stat_without_valid_servo_p:
            with self._servo_p_state_lock:
                last_valid = self._last_valid_servo_p_rx_time
            if last_valid is None or (timestamp - last_valid) > float(self.valid_servo_p_timeout_sec):
                return
        
        # Avoid duplicate collection within the minimum interval
        if timestamp - self.last_edg_collect_timestamp < self.min_edg_collect_interval:
            return
        self.last_edg_collect_timestamp = timestamp
        
        # Extract data
        left_joint_pos = self.left_arm_joint_state.position[:7] if len(self.left_arm_joint_state.position) >= 7 else [0.0] * 7
        right_joint_pos = self.right_arm_joint_state.position[:7] if len(self.right_arm_joint_state.position) >= 7 else [0.0] * 7
        
        # Extract TCP pose (x, y, z, qw, qx, qy, qz)
        left_tcp = self.left_arm_tcp_pose.position[:7] if len(self.left_arm_tcp_pose.position) >= 7 else [0.0] * 7
        right_tcp = self.right_arm_tcp_pose.position[:7] if len(self.right_arm_tcp_pose.position) >= 7 else [0.0] * 7
        
        # Convert quaternions to RPY angles in degrees
        left_rpy = [0.0, 0.0, 0.0]
        right_rpy = [0.0, 0.0, 0.0]
        if len(left_tcp) >= 7:
            try:
                quat_left = [left_tcp[4], left_tcp[5], left_tcp[6], left_tcp[3]]  # x, y, z, w
                rot_left = R.from_quat(quat_left)
                left_rpy = list(rot_left.as_euler('xyz', degrees=True))
            except:
                pass
        
        if len(right_tcp) >= 7:
            try:
                quat_right = [right_tcp[4], right_tcp[5], right_tcp[6], right_tcp[3]]  # x, y, z, w
                rot_right = R.from_quat(quat_right)
                right_rpy = list(rot_right.as_euler('xyz', degrees=True))
            except:
                pass
        
        # Extract torque data
        if self.left_arm_raw_torque is not None and len(self.left_arm_raw_torque.position) >= 6:
            left_torque = list(self.left_arm_raw_torque.position[:6])
        else:
            left_torque = [0.0] * 6
        
        if self.right_arm_raw_torque is not None and len(self.right_arm_raw_torque.position) >= 6:
            right_torque = list(self.right_arm_raw_torque.position[:6])
        else:
            right_torque = [0.0] * 6
        
        # Write the data
        if self.save_format == 'csv':
            row = [timestamp]
            # Left and right joint positions
            for i in range(7):
                row.append(left_joint_pos[i] if i < len(left_joint_pos) else 0.0)
            for i in range(7):
                row.append(right_joint_pos[i] if i < len(right_joint_pos) else 0.0)
            # Left-arm TCP pose
            row.extend([
                left_tcp[0] if len(left_tcp) > 0 else 0.0,
                left_tcp[1] if len(left_tcp) > 1 else 0.0,
                left_tcp[2] if len(left_tcp) > 2 else 0.0,
                left_rpy[0] if len(left_rpy) > 0 else 0.0,
                left_rpy[1] if len(left_rpy) > 1 else 0.0,
                left_rpy[2] if len(left_rpy) > 2 else 0.0,
            ])
            # Right-arm TCP pose
            row.extend([
                right_tcp[0] if len(right_tcp) > 0 else 0.0,
                right_tcp[1] if len(right_tcp) > 1 else 0.0,
                right_tcp[2] if len(right_tcp) > 2 else 0.0,
                right_rpy[0] if len(right_rpy) > 0 else 0.0,
                right_rpy[1] if len(right_rpy) > 1 else 0.0,
                right_rpy[2] if len(right_rpy) > 2 else 0.0,
            ])
            # Torque data
            for i in range(6):
                row.append(left_torque[i] if i < len(left_torque) else 0.0)
            for i in range(6):
                row.append(right_torque[i] if i < len(right_torque) else 0.0)
            
            self.edg_stat_writer.writerow(row)
            self.edg_stat_file_handle.flush()
            self.edg_stat_count += 1
        else:  # json
            data = {
                'timestamp': timestamp,
                'left_joint_pos': left_joint_pos,
                'right_joint_pos': right_joint_pos,
                'left_tcp_pos': list(left_tcp[:3]),
                'left_tcp_rpy': left_rpy,
                'right_tcp_pos': list(right_tcp[:3]),
                'right_tcp_rpy': right_rpy,
                'left_torque': left_torque,
                'right_torque': right_torque,
            }
            self.edg_stat_data.append(data)
            self.edg_stat_count += 1

    def print_stats(self):
        """Print statistics periodically."""
        self.get_logger().info(
            f'Recorded: servo_p={self.servo_p_count}, servo_p_filtered={self.servo_p_filtered_count}, '
            f'vr_pose={self.vr_pose_count} (left={self.left_vr_pose_count}, right={self.right_vr_pose_count}), '
            f'edg_stat={self.edg_stat_count}'
        )
    
    def save_json_data(self):
        """Save JSON data."""
        if self.save_format == 'json':
            with open(self.servo_p_file, 'w') as f:
                json.dump(self.servo_p_data, f, indent=2)
            with open(self.servo_p_filtered_file, 'w') as f:
                json.dump(self.servo_p_filtered_data, f, indent=2)
            with open(self.vr_pose_file, 'w') as f:
                json.dump(self.vr_pose_data, f, indent=2)
            with open(self.edg_stat_file, 'w') as f:
                json.dump(self.edg_stat_data, f, indent=2)
            self.get_logger().info('JSON data saved')
    
    def destroy_node(self):
        """Save data before node destruction."""
        if self.save_format == 'json':
            self.save_json_data()
        else:
            if self.servo_p_file_handle:
                self.servo_p_file_handle.close()
            if self.servo_p_filtered_file_handle:
                self.servo_p_filtered_file_handle.close()
            if self.vr_pose_file_handle:
                self.vr_pose_file_handle.close()
            if self.edg_stat_file_handle:
                self.edg_stat_file_handle.close()
        
        self.get_logger().info(
            f'Final stats: servo_p={self.servo_p_count}, servo_p_filtered={self.servo_p_filtered_count}, '
            f'vr_pose={self.vr_pose_count} (left={self.left_vr_pose_count}, right={self.right_vr_pose_count}), '
            f'edg_stat={self.edg_stat_count}'
        )
        self.get_logger().info('Data recording stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    recorder = TopicDataRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Interrupted by user')
    except Exception as e:
        recorder.get_logger().error(f'Error: {e}')
    finally:
        try:
            recorder.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

