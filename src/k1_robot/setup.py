from setuptools import find_packages, setup

package_name = 'k1_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'],include=['k1_robot', 'k1_robot.*']),
    package_data={'k1_robot': ['*.so']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pydhgripper'],
    zip_safe=True,
    maintainer='shyreckdc',
    maintainer_email='shyreckdc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_servo = k1_robot.robot_servo:main',
            'robot_interface = k1_robot.robot_interface:main',
            'test_robot_interface_vps = k1_robot.test_robot_interface_vps:main',
            'test_robot_interface_xy = k1_robot.test_robot_interface_xy:main',
            'control_gripper = k1_robot.control_gripper:main',
            'pub_pos = k1_robot.pub_set_pos:main',
        ],
    },
)
