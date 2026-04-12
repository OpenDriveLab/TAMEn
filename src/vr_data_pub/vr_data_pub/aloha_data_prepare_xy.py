import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import h5py
import numpy as np
import os
import cv2
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
import time
import sys
import message_filters
# from message_filters import ApproximateTimeSynchronizer, Subscriber
from datetime import datetime, timezone, timedelta
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R

'''
data_dict = {
    # ----------ROBOT----------- 
    '/observations/qpos': Concat[ left_arm_qpos (7),          # absolute joint position
                                  left_gripper_qpos (1),  # normalized gripper position (0: close, 1: open)
                                  right_arm_qpos (7),         # absolute joint position
                                  right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)

    '/observations/qvel': Concat[ left_arm_qvel (7),          # absolute joint velocity (rad)
                                    left_gripper_qvel (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                    right_arm_qvel (7),         # absolute joint velocity (rad)
                                    right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
    # -------VR DATA--------------
    '/action': [left_arm_pose (7),             # position and quaternion for end effector                # need calculate
                left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)         
                right_arm_pose (7),            # position and quaternion for end effector
                right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)          # need calculate
    # ---------CAMERA-------------
    '/observations/images/
        render_cam': (480x640x3)        # h, w, c, dtype='uint8'
        left_shoulder_cam': (480x640x3) # h, w, c, dtype='uint8'
        right_shoulder_cam': (480x640x3)# h, w, c, dtype='uint8'
}

'''


class DataCollector(Node):
    def __init__(self):
        super().__init__('aloha_data_prepare_node')
        self.bridge = CvBridge()
        self.image_count = 0
        self.episode_idx = 0
        self.dataset_dir = '/root/K1-W/datasets'
        os.makedirs(self.dataset_dir, exist_ok=True)
        self.target_message_count = 10
        # self.camera_names = ['right_cam', 'front_cam', 'left_cam','left_fixed_cam','right_fixed_cam']
        self.camera_names = ['right_cam', 'left_cam', 'right_tactile', 'left_tactile']
        self.data_dict1 = {
            '/observations/qpos_p': [],
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/action_p': [],
            '/action': [],
            # '/sensor': [],
            **{f'/observations/images/{cam_name}': [] for cam_name in self.camera_names},
        }
        # Double buffering
        # self.data_dict1 = {key: [] for key in self.data_dict}
        self.data_dict2 = {key: [] for key in self.data_dict1}
        self.current_buffer = 1  # 1 for data_dict1, 2 for data_dict2
        self.topic_status = {
            '/left_arm/joint_states': False,
            '/right_arm/joint_states': False,
            '/left_arm/cur_tcp_pose': False,
            '/right_arm/cur_tcp_pose': False,
            '/left_gripper/current_pos': False,
            '/right_gripper/current_pos': False,
            '/left_arm/joint_states_vel': False,
            '/right_arm/joint_states_vel': False,
            '/left_arm/robot_command_servo_p': False,
            '/right_arm/robot_command_servo_p': False,
            #'/left_arm/robot_command_servo_j': False,
            #'/right_arm/robot_command_servo_j': False,
           # '/left_arm/raw_torque_sensor_val': False,
           # '/right_arm/raw_torque_sensor_val': False,
            '/left_gripper/set_pos': False,
            '/right_gripper/set_pos': False,
            '/right_camera/color/image_raw': False,
            '/left_camera/color/image_raw': False,
            '/right_tactile_camera/color/image_raw': False,
            '/left_tactile_camera/color/image_raw': False
        }

        self.recording = False
        self.file_saved = False
        self.left_arm_qpos_j = message_filters.Subscriber(self, JointState, '/left_arm/joint_states')
        self.right_arm_qpos_j = message_filters.Subscriber(self, JointState, '/right_arm/joint_states')
        self.left_arm_qpos_p = message_filters.Subscriber(self, JointState, '/left_arm/cur_tcp_pose')
        self.right_arm_qpos_p = message_filters.Subscriber(self, JointState, '/right_arm/cur_tcp_pose')
        self.left_gripper_qpos = message_filters.Subscriber(self, JointState, '/left_gripper/current_pos')
        self.right_gripper_qpos = message_filters.Subscriber(self, JointState, '/right_gripper/current_pos')
        self.left_arm_qvel = message_filters.Subscriber(self, JointState, '/left_arm/joint_states_vel')
        self.right_arm_qvel = message_filters.Subscriber(self, JointState, '/right_arm/joint_states_vel')
        self.left_vr_servo_p_pose = message_filters.Subscriber(self, JointState, '/left_arm/robot_command_servo_p')
        self.right_vr_servo_p_pose = message_filters.Subscriber(self, JointState, '/right_arm/robot_command_servo_p')
        #self.left_vr_servo_j_pose = message_filters.Subscriber(self, JointState, '/left_arm/robot_command_servo_j')
        #self.right_vr_servo_j_pose = message_filters.Subscriber(self, JointState, '/right_arm/robot_command_servo_j')
        #self.left_sensor_val = message_filters.Subscriber(self, JointState, '/left_arm/raw_torque_sensor_val')
        #self.right_sensor_val = message_filters.Subscriber(self, JointState, '/right_arm/raw_torque_sensor_val')
        # Gripper set_pos is not synchronized; keep only the latest sample to avoid sparse button-triggered updates
        self._last_left_gripper_set_pos = None
        self._last_right_gripper_set_pos = None
        self.right_cam_sub = message_filters.Subscriber(self, Image, '/right_camera/color/image_raw')
        self.left_cam_sub = message_filters.Subscriber(self, Image, '/left_camera/color/image_raw')
        self.right_tactile_cam_sub = message_filters.Subscriber(self, Image, '/right_tactile_camera/color/image_raw')
        self.left_tactile_cam_sub = message_filters.Subscriber(self, Image, '/left_tactile_camera/color/image_raw')

        # # robot
        # qpos
        self.create_subscription(JointState, '/left_arm/joint_states', self.left_arm_joint_states_callback, 10)
        self.create_subscription(JointState, '/right_arm/joint_states', self.right_arm_joint_states_callback, 10)
        self.create_subscription(JointState, '/left_arm/cur_tcp_pose', self.left_arm_cur_tcp_pos_callback, 10)
        self.create_subscription(JointState, '/right_arm/cur_tcp_pose', self.right_arm_cur_tcp_pos_callback, 10)
        self.create_subscription(JointState, '/left_gripper/current_pos', self.left_gripper_current_pos_callback, 10)
        self.create_subscription(JointState, '/right_gripper/current_pos', self.right_gripper_current_pos_callback, 10)
        self.create_subscription(JointState, '/left_arm/joint_states_vel', self.left_arm_joint_states_vel_callback, 10)
        self.create_subscription(JointState, '/right_arm/joint_states_vel', self.right_arm_joint_states_vel_callback,
                                 10)
        self.create_subscription(JointState, '/left_arm/robot_command_servo_p',
                                 self.left_arm_robot_command_servo_p_callback, 10)
        self.create_subscription(JointState, '/right_arm/robot_command_servo_p',
                                 self.right_arm_robot_command_servo_p_callback, 10)
        # self.create_subscription(JointState, '/left_arm/robot_command_servo_j',
        #                          self.left_arm_robot_command_servo_j_callback, 10)
        # self.create_subscription(JointState, '/right_arm/robot_command_servo_j',
        #                          self.right_arm_robot_command_servo_j_callback, 10)
        #self.create_subscription(JointState, '/left_arm/raw_torque_sensor_val',
        #                         self.left_arm_raw_torque_sensor_val_callback, 10)
        #self.create_subscription(JointState, '/right_arm/raw_torque_sensor_val',
        #                         self.right_arm_raw_torque_sensor_val_callback, 10)
        self.create_subscription(JointState, '/left_gripper/set_pos', self.left_gripper_set_pos_callback, 10)
        self.create_subscription(JointState, '/right_gripper/set_pos', self.right_gripper_set_pos_callback, 10)
        self.create_subscription(Image, '/right_camera/color/image_raw', self.right_camera_image_callback, 10)
        self.create_subscription(Image, '/left_camera/color/image_raw', self.left_camera_image_callback, 10)
        self.create_subscription(Image, '/right_tactile_camera/color/image_raw', self.right_tactile_camera_image_callback, 10)
        self.create_subscription(Image, '/left_tactile_camera/color/image_raw', self.left_tactile_camera_image_callback, 10)

        # self.right_fixed_cam_sub = Subscriber(self, Image, '/right_fixed_camera/color/image_raw')
        # Time Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [
                self.left_arm_qpos_j, self.right_arm_qpos_j,
                self.left_arm_qpos_p, self.right_arm_qpos_p,
                self.left_gripper_qpos, self.right_gripper_qpos,
                self.left_arm_qvel, self.right_arm_qvel,
                self.left_vr_servo_p_pose, self.right_vr_servo_p_pose,
                self.right_cam_sub, self.left_cam_sub,
                self.right_tactile_cam_sub, self.left_tactile_cam_sub
            ],
            queue_size=1,
            slop=1,
            allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info("ApproximateTimeSynchronizer initialized and callback registered.")
        self.create_timer(5, self.check_topic_status)
        # Services
        self.start_recording_service = self.create_service(SetBool, 'start_recording', self.start_recording_callback)
        self.save_data_service = self.create_service(SetBool, 'save_data', self.save_data_callback)
        self.get_logger().info("Services for start_recording and save_data are ready.")

    def left_arm_joint_states_callback(self, msg):
        self.topic_status["/left_arm/joint_states"] = True

    # Handle left-arm joint-state messages

    def right_arm_joint_states_callback(self, msg):
        self.topic_status["/right_arm/joint_states"] = True

    # Handle right-arm joint-state messages

    def left_arm_cur_tcp_pos_callback(self, msg):
        self.topic_status["/left_arm/cur_tcp_pose"] = True
        # Handle left-arm TCP pose messages

    def right_arm_cur_tcp_pos_callback(self, msg):
        self.topic_status["/right_arm/cur_tcp_pose"] = True
        # Handle right-arm TCP pose messages

    def left_gripper_current_pos_callback(self, msg):
        self.topic_status["/left_gripper/current_pos"] = True
        # Handle left-gripper current-position messages

    def right_gripper_current_pos_callback(self, msg):
        self.topic_status["/right_gripper/current_pos"] = True
        # Handle right-gripper current-position messages

    def left_arm_joint_states_vel_callback(self, msg):
        self.topic_status["/left_arm/joint_states_vel"] = True
        # Handle left-arm joint-velocity messages

    def right_arm_joint_states_vel_callback(self, msg):
        self.topic_status["/right_arm/joint_states_vel"] = True
        # Handle right-arm joint-velocity messages

    def left_arm_robot_command_servo_p_callback(self, msg):
        self.topic_status["/left_arm/robot_command_servo_p"] = True
        # Handle left-arm position-servo command messages

    def right_arm_robot_command_servo_p_callback(self, msg):
        self.topic_status["/right_arm/robot_command_servo_p"] = True
        # Handle right-arm position-servo command messages

    # def left_arm_robot_command_servo_j_callback(self, msg):
    #     self.topic_status["/left_arm/robot_command_servo_j"] = True
    #     # Handle left-arm joint-servo command messages

    # def right_arm_robot_command_servo_j_callback(self, msg):
    #     self.topic_status["/right_arm/robot_command_servo_j"] = True
        # Handle right-arm joint-servo command messages

    #def left_arm_raw_torque_sensor_val_callback(self, msg):
    #    self.topic_status["/left_arm/raw_torque_sensor_val"] = True
        # Handle left-arm torque-sensor messages

    #def right_arm_raw_torque_sensor_val_callback(self, msg):
    #   self.topic_status["/right_arm/raw_torque_sensor_val"] = True
        # Handle right-arm torque-sensor messages

    def left_gripper_set_pos_callback(self, msg):
        self.topic_status["/left_gripper/set_pos"] = True
        self._last_left_gripper_set_pos = msg

    def right_gripper_set_pos_callback(self, msg):
        self.topic_status["/right_gripper/set_pos"] = True
        self._last_right_gripper_set_pos = msg

    def right_camera_image_callback(self, msg):
        self.topic_status["/right_camera/color/image_raw"] = True
        # Handle right-camera image messages

    def left_camera_image_callback(self, msg):
        self.topic_status["/left_camera/color/image_raw"] = True

    def right_tactile_camera_image_callback(self, msg):
        self.topic_status["/right_tactile_camera/color/image_raw"] = True

    def left_tactile_camera_image_callback(self, msg):
        self.topic_status["/left_tactile_camera/color/image_raw"] = True

    def check_topic_status(self):
        for topic, status in self.topic_status.items():
            if not status:
                self.get_logger().error(f"ERROR: No data received from {topic}!")

    def start_recording_callback(self, request, response):
        if request.data:
            self.recording = True
            # self.file_saved = False
            self.get_logger().info('Start recording data.')
        response.success = True  # Service response object used to return the result
        return response

    def save_data_callback(self, request, response):
        if request.data and self.recording and not self.file_saved:
            self.saving = True  #
            self.file_saved = True

            current_dict = self.data_dict1 if self.current_buffer == 1 else self.data_dict2

            self.save_hdf5(current_dict)

            self.current_buffer = 2 if self.current_buffer == 1 else 1  # Switch buffers
            self.data_dict1 = {key: [] for key in self.data_dict1}  # Reset the current buffer
            self.data_dict2 = {key: [] for key in self.data_dict2}  # Reset the other buffer
            self.saving = False

            self.recording = False
            self.file_saved = False
            # self.data_dict = {key: [] for key in self.data_dict}  # reset

            self.get_logger().info('Data saved and reset for next recording.')
        response.success = True
        return response

    def sync_callback(self, left_arm_qpos_j_msg, right_arm_qpos_j_msg,
                      left_arm_qpos_p_msg, right_arm_qpos_p_msg,
                      left_gripper_msg, right_gripper_msg,
                      left_arm_qvel_msg, right_arm_qvel_msg,
                      left_vr_pose_p_msg, right_vr_pose_p_msg,
                      right_cam_msg, left_cam_msg,
                      right_tactile_cam_msg, left_tactile_cam_msg):
        if not self.recording:
            return
        # Use the latest cached gripper set_pos; fall back to current_pos if no command has arrived yet
        left_vr_gripper_msg = self._last_left_gripper_set_pos if self._last_left_gripper_set_pos is not None else left_gripper_msg
        right_vr_gripper_msg = self._last_right_gripper_set_pos if self._last_right_gripper_set_pos is not None else right_gripper_msg

        current_dict = self.data_dict1 if self.current_buffer == 1 else self.data_dict2
        self.process_qpos(left_arm_qpos_j_msg, right_arm_qpos_j_msg,
                          left_arm_qpos_p_msg, right_arm_qpos_p_msg,
                          left_gripper_msg, right_gripper_msg, current_dict)
        self.process_qvel(left_arm_qvel_msg, right_arm_qvel_msg,
                          left_gripper_msg, right_gripper_msg, current_dict)
        self.process_action(left_arm_qpos_j_msg, right_arm_qpos_j_msg,
                            left_vr_pose_p_msg, right_vr_pose_p_msg,
                            left_vr_gripper_msg, right_vr_gripper_msg, current_dict)
        self.process_images(right_cam_msg, left_cam_msg,
                            right_tactile_cam_msg, left_tactile_cam_msg,
                            current_dict)

    def process_qpos(self, left_arm_qpos_j_msg, right_arm_qpos_j_msg,
                     left_arm_qpos_p_msg, right_arm_qpos_p_msg,
                    # left_sensor_msg, right_sensor_msg,
                     left_gripper_msg, right_gripper_msg, current_dict
                     ):
        print('1. process_qpos')
        left_arm_qpos_j = left_arm_qpos_j_msg.position[:7]
        right_arm_qpos_j = right_arm_qpos_j_msg.position[:7]
        # Gripper: linearly map 0-1 to 0.03 + 0.145 * x
        left_gripper_qpos = 0.03 + 0.145 * left_gripper_msg.position[0]
        right_gripper_qpos = 0.03 + 0.145 * right_gripper_msg.position[0]
        qpos = np.concatenate([
            left_arm_qpos_j,
            np.array([left_gripper_qpos]),
            right_arm_qpos_j,
            np.array([right_gripper_qpos]),
        ])
        current_dict['/observations/qpos'].append(qpos)
        print('/observations/qpos:', len(current_dict['/observations/qpos']))
        left_rotation = R.from_quat(left_arm_qpos_p_msg.position[-4:])
        right_rotation = R.from_quat(right_arm_qpos_p_msg.position[-4:])
        left_euler_angles = left_rotation.as_euler('xyz', degrees=False)
        right_euler_angles = right_rotation.as_euler('xyz', degrees=False)
        left_arm_qpos_p = left_arm_qpos_p_msg.position[:3]
        right_arm_qpos_p = right_arm_qpos_p_msg.position[:3]
        qpos = np.concatenate([
            left_arm_qpos_p,
            left_euler_angles,
            np.array([left_gripper_qpos]),
            right_arm_qpos_p,
            right_euler_angles,
            np.array([right_gripper_qpos]),
        ])
        current_dict['/observations/qpos_p'].append(qpos)
        print('/observations/qpos_p:', len(current_dict['/observations/qpos_p']))

    def process_qvel(self, left_arm_qvel_msg, right_arm_qvel_msg,
                     left_gripper_msg, right_gripper_msg, current_dict
                     ):
        print('2. process_qvel')
        left_arm_qvel = left_arm_qvel_msg.velocity[:7]
        right_arm_qvel = right_arm_qvel_msg.velocity[:7]
        left_gripper_qvel = left_gripper_msg.position[0]  # Assume gripper qvel equals qpos
        right_gripper_qvel = right_gripper_msg.position[0]  # Assume gripper qvel equals qpos
        qvel = np.concatenate([
            left_arm_qvel,
            np.array([left_gripper_qvel]),
            right_arm_qvel,
            np.array([right_gripper_qvel])
        ])
        current_dict['/observations/qvel'].append(qvel)
        print('/observations/qvel:', len(current_dict['/observations/qvel']))

    def process_action(self, 
                        left_arm_qpos_j_msg, right_arm_qpos_j_msg,
                        left_vr_pose_p_msg, right_vr_pose_p_msg,
                    #    left_vr_pose_j_msg, right_vr_pose_j_msg,
                     #  left_sensor_msg, right_sensor_msg,
                       left_vr_gripper_msg, right_vr_gripper_msg, current_dict
                       ):
        print('3. process_action')
        left_vr_pose_p = left_vr_pose_p_msg.position[:6]
        right_vr_pose_p = right_vr_pose_p_msg.position[:6]
        # Gripper: linearly map 0-1 to 0.03 + 0.145 * x
        left_vr_gripper = 0.03 + 0.145 * left_vr_gripper_msg.position[0]
        right_vr_gripper = 0.03 + 0.145 * right_vr_gripper_msg.position[0]
        action_p = np.concatenate(
            [left_vr_pose_p, np.array([left_vr_gripper]), right_vr_pose_p, np.array([right_vr_gripper])])
        current_dict['/action_p'].append(action_p)
        print('/action_p:', len(current_dict['/action_p']))
        left_vr_pose_j = left_arm_qpos_j_msg.position[:7]
        right_vr_pose_j = right_arm_qpos_j_msg.position[:7]
        action2 = np.concatenate(
            [left_vr_pose_j, np.array([left_vr_gripper]), right_vr_pose_j, np.array([right_vr_gripper])])
        current_dict['/action'].append(action2)
        print('/action', len(current_dict['/action']))
        # sensor = left_sensor + right_sensor
        # current_dict['/sensor'].append(sensor)
        # print('/sensor:', len(current_dict['/sensor']))

    def process_images(self, right_cam_msg, left_cam_msg,
                       right_tactile_cam_msg, left_tactile_cam_msg,
                       current_dict):
        for cam_msg, cam_name in zip([right_cam_msg, left_cam_msg,
                                      right_tactile_cam_msg, left_tactile_cam_msg
                                      ], self.camera_names):
            img = self.bridge.imgmsg_to_cv2(cam_msg, desired_encoding='passthrough')
            img_resized = cv2.resize(img, (640, 480), interpolation=cv2.INTER_LANCZOS4)
            img = np.array(img_resized, dtype=np.uint8)
            current_dict[f'/observations/images/{cam_name}'].append(img)
        print('images appended!')

    def save_hdf5(self, current_dict):
        beijing_tz = timezone(timedelta(hours=8))
        timestamp = datetime.now(beijing_tz).strftime("%Y%m%d-%H%M%S")
        dataset_path = os.path.join(self.dataset_dir, f'episode_{timestamp}')
        max_timesteps = len(current_dict['/observations/images/left_cam'])
        print(f"Saving {max_timesteps} timesteps to {dataset_path}.hdf5")
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            obs = root.create_group('observations')
            qpos_p = obs.create_dataset('qpos_p', (max_timesteps, 14), dtype='float32',
                                        data=np.array(current_dict['/observations/qpos_p'], dtype=np.float32))  # 6 + 6 + 1 + 1
            qpos_j = obs.create_dataset('qpos', (max_timesteps, 16), dtype='float32',
                                        data=np.array(current_dict['/observations/qpos'], dtype=np.float32))  # 7 + 7 + 1 + 1
            qvel = obs.create_dataset('qvel', (max_timesteps, 16), dtype='float32',
                                      data=np.array(current_dict['/observations/qvel'], dtype=np.float32))  # 16
            action_p = root.create_dataset('action_p', (max_timesteps, 14), dtype='float32',
                                           data=np.array(current_dict['/action_p'], dtype=np.float32))  # 6 + 6 + 1 + 1
            action_j = root.create_dataset('action', (max_timesteps, 16), dtype='float32',
                                           data=np.array(current_dict['/action'], dtype=np.float32))  # 7 + 7 + 1 + 1
            # sensor = root.create_dataset('sensor', (max_timesteps, 12), data=current_dict['/sensor'])  # 6 + 6
            image = obs.create_group('images')
            # print('image:',image)
            for cam_name in self.camera_names:
                image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                     data=current_dict[f'/observations/images/{cam_name}'])


def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()


class Logger(object):
    def __init__(self, file_name='temp.log', stream=sys.stdout) -> None:
        self.terminal = stream
        current_time = time.strftime('%Y-%m-%d_%H-%M-%S')
        self.log_file_name = f'log/{file_name}_{current_time}.log'
        if not os.path.exists('log'):
            os.makedirs('log')
        self.log = open(self.log_file_name, "a")

    def flush(self):
        self.log.flush()


if __name__ == '__main__':
    logger = Logger()
    sys.stdout = logger
    sys.stderr = logger
    main()
