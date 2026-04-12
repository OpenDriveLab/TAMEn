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
        super().__init__('vr_robot_pose_converter')
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
            PoseStamped,
            'cur_tcp_pose',
            self.cur_tcp_pose_callback,
            10)
        self.cur_tcp_pose_sub # prevent unused variable warning

        # Publisher to send servo_j7 commands
        self.publisher_servo_j = self.create_publisher(JointState, 'robot_command_servo_j', 10)
        self.publisher_test = self.create_publisher(String, 'test_fre', 10)

        self.vr_robot_pose_converter_service = self.create_service(SetBool, 'vr_robot_pose_converter', self.vr_robot_pose_converter_callback)
        self.vr_robot_pose_converter_status = False


        self.kine_inverse_client = self.create_client(KineInverse7, 'kine_inverse7')
        while not self.kine_inverse_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service kine_inverse7 not available, waiting again...')

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
        self.cur_tcp_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # self.get_logger().info(f'update pos: {self.cur_tcp_position}')

        # Create a Rotation object from the quaternion (x, y, z, w)
        rotation = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # Get the rotation matrix
        self.cur_tcp_rotm = rotation.as_matrix()
        # self.get_logger().info(f'update rot: {self.cur_tcp_rotm}')

    def vr_left_pose_callback(self, msg):
        if self.vr_robot_pose_converter_status==True:
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

            vr_rot = np.array([
                [0, 0, -1],
                [-1, 0, 0],
                [0, 1, 0]
            ])

            left_diff_base= np.dot(vr_rot,np.dot(left_rotvr_diff,np.linalg.inv(vr_rot)))

            # move_self.left_robot =False
            # if move_self.left_robot:
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
                euler_angles_xyz_fixed = rotation.as_euler('xyz', degrees=False)
                next_pose = [0.0]*6
                next_pose[0] = self.leftarm_init_pos[0] + left_diff[2]
                next_pose[1] = self.leftarm_init_pos[1] - left_diff[0]
                next_pose[2] = self.leftarm_init_pos[2] + left_diff[1]

                next_pose[3] = euler_angles_xyz_fixed[0]
                next_pose[4] = euler_angles_xyz_fixed[1]
                next_pose[5] = euler_angles_xyz_fixed[2]

                # print('L_reftPOS:',left_ref_pos)
                # print('L_nextPOS:',next_pose)

                 # Call kine_inverse7 service
                self.get_logger().warn("call ik")
                new_msg = String()
                new_msg.data = str("hello")
                self.publisher_test.publish(new_msg)
                future = self.call_kine_inverse7(self.cur_joint_val,next_pose)
                future.add_done_callback(self.handle_kine_inverse7_response)
                self.get_logger().warn("after ik")
            


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