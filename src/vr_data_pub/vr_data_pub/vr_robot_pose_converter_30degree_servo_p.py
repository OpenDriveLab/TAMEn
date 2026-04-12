import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from k1_msgs.srv import JointMove, LinearMove, KineInverse7
from std_srvs.srv import SetBool

import re
import math
import numpy as np


class VrRobotPoseConverter(Node):

    def __init__(self):
        super().__init__('vr_robot_pose_converter_30rad')
        self.vr_left_pose_sub = self.create_subscription(
            String,
            'vr_pose',
            self.vr_left_pose_callback,
            10)
        self.vr_left_pose_sub # prevent unused variable warning

        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.joint_states_sub # prevent unused variable warning

        self.cur_tcp_pose_sub = self.create_subscription(
            JointState,
            'cur_tcp_pose',
            self.cur_tcp_pose_callback,
            10)
        self.cur_tcp_pose_sub # prevent unused variable warning

        # Publisher to send servo_j7 commands
        self.publisher_servo_j = self.create_publisher(JointState, 'robot_command_servo_j', 1)
        self.publisher_servo_p = self.create_publisher(JointState, 'robot_command_servo_p', 1)
        self.publisher_test = self.create_publisher(String, 'test_fre', 10)

        self.vr_robot_pose_converter_service = self.create_service(SetBool, 'vr_robot_pose_converter', self.vr_robot_pose_converter_callback)
        self.vr_robot_pose_converter_status = False


        # self.kine_inverse_client = self.create_client(KineInverse7, 'kine_inverse7')
        # while not self.kine_inverse_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service kine_inverse7 not available, waiting again...')

        self.cur_joint_val = None
        self.cur_tcp_position = None
        self.cur_tcp_rotm = None


        self.ABS = 0
        self.INCR = 1
        self.ENABLE = True
        self.DISABLE = False
        self.SPEEDTHRE = 180

        self.init_left_vr_pose = None
        self.leftarm_init_rot = None
        self.leftarm_init_pos = None
        self.declare_parameter('robot_name', 'left')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
    
        # Used for angle continuity: store the previous Euler angles
        self.last_euler_angles = None
    
    def vr_robot_pose_converter_callback(self, request, response):
        """Service callback used to enable or disable SERVO MOVE mode."""
        try:
            status = request.data
            if not status:
                self.cur_joint_val = None
                self.cur_tcp_position = None
                self.cur_tcp_rotm = None
                self.init_left_vr_pose = None
                self.leftarm_init_rot = None
                self.leftarm_init_pos = None
                self.last_euler_angles = None  # Reset angle history
            self.vr_robot_pose_converter_status = status # Update internal state
            response.success = 0
            response.message = "Enabled vr_robot_pose_converter" if status else "Disabled vr_robot_pose_converter"
            if not response.success:
                response.message = f"Error: {status}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def euler_to_rotation_matrix(self, rx, ry, rz):
        # rx, ry, rz = np.radians([rx, ry, rz])
        # Compute the rotation matrix
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
    
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
    
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        # Combine the three rotation matrices
        # R = np.dot(Rz, np.dot(Rx, Ry))
        R = np.dot(Ry, np.dot(Rx, Rz))
        return R 
    
    def rotation_matrix_z(self,angle):
        theta = np.radians(angle)
        return np.array([
            [np.cos(theta),-np.sin(theta),0],
            [np.sin(theta),np.cos(theta),0],
            [0,0,1]
        ])

    def scale_rotation_matrix_axis_angle(self, rotation_matrix, scale_factor):
        """
        Scale a rotation matrix by scaling its axis-angle representation.

        Args:
        rotation_matrix: 3x3 rotation matrix
        scale_factor: scale factor (0-1 reduces rotation, >1 amplifies it)

        Returns:
        The scaled rotation matrix
        """
        # Convert the rotation matrix to axis-angle form with scipy
        rotation = R.from_matrix(rotation_matrix)
        
        # Get the rotation vector (axis-angle form: direction is axis, length is angle)
        rotvec = rotation.as_rotvec()
        
        # Scale the rotation angle while keeping the axis direction unchanged
        scaled_rotvec = rotvec * scale_factor
        
        # Convert the scaled axis-angle back to a rotation matrix
        scaled_rotation = R.from_rotvec(scaled_rotvec)
        scaled_matrix = scaled_rotation.as_matrix()
        
        return scaled_matrix
    
    def normalize_angle_continuous(self, angle, prev_angle=None):
        """
        Normalize an angle to [-pi, pi] and keep it continuous with the previous angle.

        Args:
        angle: current angle in radians
        prev_angle: previous angle in radians, used for continuity

        Returns:
        The normalized continuous angle
        """
        # First normalize to [-pi, pi]
        normalized = ((angle + np.pi) % (2 * np.pi)) - np.pi
        
        # If a previous angle exists, preserve continuity and avoid +-pi jumps
        if prev_angle is not None:
            # Compute the difference between the normalized angle and the previous angle
            diff = normalized - prev_angle
            
            # If the difference exceeds pi, a discontinuity occurred and must be corrected
            if diff > np.pi:
                normalized -= 2 * np.pi
            elif diff < -np.pi:
                normalized += 2 * np.pi
        
        return normalized
    
    def normalize_euler_angles_continuous(self, euler_angles, prev_euler_angles=None):
        """
        Normalize an Euler-angle array while preserving continuity and avoiding +-pi jumps.

        Args:
        euler_angles: current Euler angles [rx, ry, rz] in radians
        prev_euler_angles: previous Euler angles [rx, ry, rz] in radians

        Returns:
        The normalized continuous Euler angles
        """
        normalized = np.zeros(3)
        
        if prev_euler_angles is None:
            # First sample: only perform basic normalization
            for i in range(3):
                normalized[i] = self.normalize_angle_continuous(euler_angles[i])
        else:
            # Subsequent samples: preserve continuity
            for i in range(3):
                normalized[i] = self.normalize_angle_continuous(euler_angles[i], prev_euler_angles[i])
        
        return normalized

    

#    def call_kine_inverse7(self, ref_joint_pos, cartesian_pose):
#        request = KineInverse7.Request()
#        request.ref_joint_pos = ref_joint_pos
#        request.cartesian_pose = cartesian_pose
#        future = self.kine_inverse_client.call_async(request)
#        rclpy.spin_until_future_complete(self, future)
#        return future.result()
    
    def call_kine_inverse7(self, ref_joint_pos, cartesian_pose):
        request = KineInverse7.Request()
        request.ref_joint_pos = ref_joint_pos
        request.cartesian_pose = cartesian_pose
        future = self.kine_inverse_client.call_async(request)
        return future  # Return the future without waiting for completion
    
    def publish_servo_j7_command(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        msg.position = joint_positions
        self.publisher_servo_j.publish(msg)
    
    def publish_servo_p_command(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = joint_positions
        self.publisher_servo_p.publish(msg)
    
    def handle_kine_inverse7_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Kine Inverse Response: {response.message}")
            print(f"Kine Inverse Response: {response.message}")
            if response.success:
                print(f"Joint Positions: {response.joint_positions}")
                left_arm_command_joint = response.joint_positions



                # print('222---IK solved successfully')
                # data_plot['lpoints_suc'].append(next_pose[:3])
                larm_limit_min = [-360, -105, -360, -145, -360, -105,-360]
                larm_limit_max = [360, 105, 360, 30, 360, 105, 360]

                # # Reduce by 1 degree and convert to radians, keeping 4 decimals
                # larm_limit_min_rad = [round(math.radians(degree - 1), 4) for degree in larm_limit_min]
                # larm_limit_max_rad = [round(math.radians(degree - 1), 4) for degree in larm_limit_max]
                larm_limit_min_rad = [math.radians(degree-5) for degree in larm_limit_min]
                larm_limit_max_rad = [math.radians(degree-5) for degree in larm_limit_max]
  
                # print('left_ori--',self.left_robot_ret[1])
                larm_ret= list(left_arm_command_joint)
                for i in range(len(larm_ret)):
                    if larm_ret[i] > larm_limit_max_rad[i]:  
                        larm_ret[i] = larm_limit_max_rad[i]
                    elif larm_ret[i] < larm_limit_min_rad[i]:  
                        larm_ret[i] = larm_limit_min_rad[i]  
                self.get_logger().info(f'command joint val: {larm_ret}')
                self.publish_servo_j7_command(larm_ret)
                        

#                # Speed limiting with an intermediate value
#                l_overspeed = False
#                l_interpolated_pos = larm_ret
#                l_diff_5 = abs(larm_ret[4] - left_ref_pos[1][4])
#                l_diff_1 = abs(larm_ret[0] - left_ref_pos[1][0])
#                if l_diff_5 >= np.radians(self.SPEEDTHRE):
#                    l_overspeed = True
#                    l_interpolated_value = (larm_ret[4] + left_ref_pos[1][4]) / 2
#                    l_interpolated_pos[4] = l_interpolated_value
#                if l_diff_1 >= np.radians(self.SPEEDTHRE):
#                    l_overspeed = True
#                    l_interpolated_value_0 = (larm_ret[0] + left_ref_pos[1][0]) / 2
#                    l_interpolated_pos[0] = l_interpolated_value_0
            
      
#                if l_overspeed==True:
#                    left_servoret = self.left_robot.servo_j7_extend(l_interpolated_pos,self.ABS,5)

                # left_servoret = self.left_robot.servo_j7(larm_ret,ABS) # Joint limit protection
                # left_servoret = self.left_robot.servo_j7_extend(larm_ret,self.ABS,5) # Joint limit protection
                
            
            else:
                self.get_logger().warn("ik failed")
#            if response.success:
#                self.get_logger().info(f"Joint Positions: {response.joint_positions}")
#                # Handle a successful IK response...
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    def joint_states_callback(self, msg):
        self.cur_joint_val = list(msg.position)

    def cur_tcp_pose_callback(self, msg):
        self.cur_tcp_position = [msg.position[0],msg.position[1],msg.position[2]]
                                #  pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # self.get_logger().info(f'update pos: {self.cur_tcp_position}')

        # Create a Rotation object from the quaternion (x, y, z, w)
        rotation = R.from_quat([msg.position[4],msg.position[5],msg.position[6],msg.position[3]])
        # rotation = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # Get the rotation matrix
        self.cur_tcp_rotm = rotation.as_matrix()
        # self.get_logger().info(f'update rot: {self.cur_tcp_rotm}')

    def vr_left_pose_callback(self, msg):
        self.get_logger().warn("in pos callback, before converter status")
        if self.vr_robot_pose_converter_status==True:
            self.get_logger().warn("in pos callback, after converter status")
            # self.get_logger().info('I heard: "%s"' % msg.data)
            left_vr_data = re.findall(r'[-+]?\d*\.\d+|\d+', msg.data)
            if len(left_vr_data) < 6:
                # print("Not enough data to process.")
                return
            left_vr_data = [float(num) for num in left_vr_data]
            # meter_to_millimeter = 200
            meter_to_millimeter = 1000
            left_pos = [x * meter_to_millimeter for x in left_vr_data[:3]]
            left_rot_rad = [math.radians(angle) for angle in left_vr_data[3:6]]
            left_vr_pose = left_pos + left_rot_rad
            self.get_logger().info(f'Received numpy float array: {left_vr_pose}')
            if not self.init_left_vr_pose:
                self.get_logger().warn("init left_vr_pose")
                self.init_left_vr_pose = left_vr_pose[:]

            left_diff = [a - b for a, b in zip(left_vr_pose, self.init_left_vr_pose)]

            left_init_rot = self.euler_to_rotation_matrix(-self.init_left_vr_pose[3], -self.init_left_vr_pose[4], self.init_left_vr_pose[5])  #yxz

            left_rot = self.euler_to_rotation_matrix(-left_vr_pose[3], -left_vr_pose[4], left_vr_pose[5])

            left_rotvr_diff = np.dot(left_rot, np.linalg.inv(left_init_rot))

            # Optional: scale the rotation matrix through axis-angle scaling
            # For example, 0.5 halves the rotation angle and 2.0 doubles it
            rotation_scale_factor = 1.0  # No scaling by default; adjust as needed
            if rotation_scale_factor != 1.0:
                left_rotvr_diff = self.scale_rotation_matrix_axis_angle(left_rotvr_diff, rotation_scale_factor)

            vr_rot = np.array([
                [0, 0, -1],
                [-1, 0, 0],
                [0, 1, 0]
            ])
            left_diff_base= np.dot(vr_rot,np.dot(left_rotvr_diff,np.linalg.inv(vr_rot)))

            if self.robot_name=='left':
                z_rot = self.rotation_matrix_z(-30)
                # z_rot = self.rotation_matrix_z(45)
                vr_rot = np.dot(z_rot,vr_rot)
                left_diff_base= np.dot(vr_rot,np.dot(left_rotvr_diff,np.linalg.inv(vr_rot)))
                print("1**********************")
            elif self.robot_name=='right':
                # z_rot = self.rotation_matrix_z(45)
                z_rot = self.rotation_matrix_z(-30)
                vr_rot = np.dot(z_rot,vr_rot)
                left_diff_base= np.dot(vr_rot,np.dot(left_rotvr_diff,np.linalg.inv(vr_rot)))
                print("2**********************")




            # move_self.left_robot =False
            # if move_self.left_robot:

            self.get_logger().warn("in pos callback, before invoke ik")
            if len(self.cur_joint_val)>0 and len(self.cur_tcp_position)>0 and len(self.cur_tcp_rotm)>0 :
            # if True:
                if self.leftarm_init_rot is None:
                    self.get_logger().warn("init leftarm_init_rot")
                    self.leftarm_init_pos = self.cur_tcp_position
                    self.leftarm_init_rot = self.cur_tcp_rotm

                leftarm_finalrot = np.dot(left_diff_base,self.leftarm_init_rot) 
                # left_rot_diff = self.left_robot.rot_matrix_to_rpy(leftarm_finalrot)
                rotation = R.from_matrix(leftarm_finalrot)
                # Convert the rotation matrix to XYZ Euler angles in the fixed frame
                euler_angles_xyz_fixed_raw = rotation.as_euler('xyz', degrees=False)
                
                # Normalize angles while preserving continuity and avoiding +-pi jumps
                euler_angles_xyz_fixed = self.normalize_euler_angles_continuous(
                    euler_angles_xyz_fixed_raw, 
                    self.last_euler_angles
                )
                # Update the previous angle state
                self.last_euler_angles = euler_angles_xyz_fixed.copy()
                
                next_pose = [0.0]*6
                next_pose[0] = self.leftarm_init_pos[0] + left_diff[2]
                next_pose[1] = self.leftarm_init_pos[1] - left_diff[0]
                next_pose[2] = self.leftarm_init_pos[2] + left_diff[1]

                next_pose[3] = euler_angles_xyz_fixed[0]
                next_pose[4] = euler_angles_xyz_fixed[1]
                next_pose[5] = euler_angles_xyz_fixed[2]

                # print('L_reftPOS:',left_ref_pos)
                print("left diff:", left_diff)
                print('init pose:', self.leftarm_init_pos)
                print('L_nextPOS:',next_pose)
                self.publish_servo_p_command(next_pose)

                new_msg = String()
                new_msg.data = str("hello")
                self.publisher_test.publish(new_msg)

#                 # Call kine_inverse7 service
#                self.get_logger().warn("call ik")
#                future = self.call_kine_inverse7(self.cur_joint_val,next_pose)
#                future.add_done_callback(self.handle_kine_inverse7_response)
#                self.get_logger().warn("after ik")
            


def main(args=None):
    rclpy.init(args=args)

    vr_robot_pose_converter = VrRobotPoseConverter()
#    response = vr_robot_pose_converter.call_kine_inverse7([-0.6481058452534466,-1.0976838391287562,0.7513068543219475,
#                                         -1.2061648442642827,0.4742115084376801,1.0480659688117935,-0.1028835738311657347], [477.0, 157.0, -25.0, -3.14, 0.0, 0.0])
#    print(f"Kine Inverse Response: {response.message}")
#    if response.success:
#        print(f"Joint Positions: {response.joint_positions}")

    rclpy.spin(vr_robot_pose_converter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vr_robot_pose_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()