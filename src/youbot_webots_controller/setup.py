from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'youbot_webots_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/ui', glob('ui/*.ui')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
    ],
    install_requires=['setuptools', 'sensor_msgs', 'cv_bridge', 'numpy'],
    zip_safe=True,
    maintainer='egor',
    maintainer_email='egor@todo.todo',
    description='ROS 2 controller for KUKA YouBot in Webots',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'webots_ros2_bridge = youbot_webots_controller.webots_ros_bridge.webots_ros2_bridge_node:main',
            'platform_velocity_controller = youbot_webots_controller.platform_velocity_controller.platform_velocity_controller_node:main',
            'youbot_arm_controller = youbot_webots_controller.youbot_arm_controller.youbot_arm_controller_node:main',
            'gripper_controller = youbot_webots_controller.gripper_controller.gripper_controller_node:main',
            'gui_controller = youbot_webots_controller.gui_controller.gui_controller_node:main',
            'youbot_odometry = youbot_webots_controller.youbot_odometry.youbot_odometry_node:main',
            'image_processor = youbot_webots_controller.object_detector.image_processor_node:main',
            'triangulator_node = youbot_webots_controller.triangulator.triangulator_node:main',
        ],
    },
)