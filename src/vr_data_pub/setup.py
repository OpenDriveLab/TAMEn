'''
Author: Ding Cheng  Email: cheng.ding@jaka.com
Date: 2025-04-10 17:18:55
LastEditors: Ding Cheng  Email: cheng.ding@jaka.com
LastEditTime: 2025-04-15 16:58:13
FilePath: /K1-W/src/vr_data_pub/setup.py
Description: 

Copyright (c) 2025 by JAKA Robotics Co., Ltd. , All Rights Reserved. 
'''
from setuptools import find_packages, setup
from glob import glob

package_name = 'vr_data_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # ('share/' + package_name + '/launch', ['launch/teleoperation_system_30degree_launch_vps.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shyreckdc',
    maintainer_email='shyreckdc@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_data_pub = vr_data_pub.vr_data_pub_long_connect:main',
            'vr_data_distributer = vr_data_pub.vr_data_distributer:main',
            # 'teleoperation_control = vr_data_pub.teleoperation_control:main',
            'teleoperation_control_30degree_xy = vr_data_pub.teleoperation_control_30degree_xy:main',
            'teleoperation_control_30degree_grmg = vr_data_pub.teleoperation_control_30degree_grmg:main',
            'teleoperation_control_30degree_vps = vr_data_pub.teleoperation_control_30degree_vps:main',
            'teleoperation_control_template = vr_data_pub.teleoperation_control_template:main',
            'vr_robot_pose_converter = vr_data_pub.vr_robot_pose_converter:main',
            'collect_data_xy = vr_data_pub.aloha_data_prepare_xy:main',
            'collect_data_gr_mg = vr_data_pub.aloha_data_prepare_GRMG:main',
            'vr_robot_pose_converter_30degree = vr_data_pub.vr_robot_pose_converter_30degree:main',
            'vr_robot_pose_converter_30degree_servo_p = vr_data_pub.vr_robot_pose_converter_30degree_servo_p:main',
            'pose_interpolator = vr_data_pub.pose_interpolator:main',
            'topic_data_recorder = vr_data_pub.topic_data_recorder:main',
        ],
    },
)
