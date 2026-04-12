import rclpy
from rclpy.node import Node

from k1_robot import jkrc

from std_msgs.msg import String

import math
import re
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, '/test_topic', 10)
        timer_period = 0.008  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.ABS = 0
        self.INCR = 1
        self.ENABLE = True
        self.DISABLE = False
        self.SPEEDTHRE = 180

        self.left_robot = jkrc.RC("192.168.2.222") 
        res = self.left_robot.login()
        if res[0] != 0:
            raise "left robot login failed."
        else:
            self.get_logger().info('left robot login success')

        res = self.left_robot.power_on()
        if res[0] != 0:
            raise "left robot power_on failed."
        else:
            self.get_logger().info('left robot power_on success')

        res = self.left_robot.enable_robot()
        if res[0] != 0:
            raise "left robot enable_robot failed."
        else:
            self.get_logger().info('left robot enable_robot success')

        res = self.left_robot.servo_move_enable(self.ENABLE)
        if res[0] != 0:
            raise "left robot servo_move_enable failed."
        else:
            self.get_logger().info('left robot servo_move_enable success')
    
        self.subscription = self.create_subscription(
                String,
                '/vr_left_pose',
                self.listener_callback,
                10)
        self.subscription  # prevent unused variable warning
        self.init_left_vr_pose = None
        self.leftarm_init_rot = None
        self.leftarm_init_pos = None
        _,self.robot_command_joint = self.left_robot.get_joint_position()  
    
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

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
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
        if True:
            left_ref_pos = self.left_robot.get_joint_position()  
            self.get_logger().info(f'Received robot ref joint array: {left_ref_pos}')
            left_tcp_pos = self.left_robot.get_tcp_position()  
            cur_pose = left_tcp_pos[1].copy()
            left_pos = cur_pose[:3]
            left_rpy = cur_pose[3:6]

            if not self.leftarm_init_rot:
                self.get_logger().warn("init leftarm_init_rot")
                self.leftarm_init_pos = left_pos
                self.leftarm_init_rot = self.left_robot.rpy_to_rot_matrix(left_rpy)

            leftarm_finalrot = np.dot(left_diff_base,self.leftarm_init_rot[1]) 
            left_rot_diff = self.left_robot.rot_matrix_to_rpy(leftarm_finalrot)
            next_pose = cur_pose.copy()
            next_pose[0] = self.leftarm_init_pos[0] + left_diff[2]
            next_pose[1] = self.leftarm_init_pos[1] - left_diff[0]
            next_pose[2] = self.leftarm_init_pos[2] + left_diff[1]

            next_pose[3] = left_rot_diff[1][0]
            next_pose[4] = left_rot_diff[1][1]
            next_pose[5] = left_rot_diff[1][2]

            # print('L_reftPOS:',left_ref_pos)
            # print('L_nextPOS:',next_pose)

            left_robot_ret = self.left_robot.kine_inverse7(left_ref_pos[1],next_pose)


            if left_robot_ret[0] == 0:

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
                larm_ret= list(left_robot_ret[1])
                for i in range(len(larm_ret)):
                    if larm_ret[i] > larm_limit_max_rad[i]:  
                        larm_ret[i] = larm_limit_max_rad[i]
                    elif larm_ret[i] < larm_limit_min_rad[i]:  
                        larm_ret[i] = larm_limit_min_rad[i]  
                # print('larm_ret',larm_ret)
                        

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
                print("robot joint val", larm_ret)
                self.robot_command_joint = larm_ret
                new_msg = String()
                new_msg.data = str(larm_ret)
                self.publisher_.publish(new_msg)
                
            
            else:
                print('222-IK failed, ret:',left_robot_ret[0])


    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        left_servoret = self.left_robot.servo_j7(joint_pos=self.robot_command_joint,move_mode = self.ABS) # Joint limit protection
#        if(self.i < 1000):
#            self.left_robot.servo_j7(joint_pos =[0.001,0,0,0,0,0.001,0],move_mode = self.INCR)
#        else:
#            self.left_robot.servo_move_enable(self.DISABLE)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()