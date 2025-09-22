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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
    ],
    install_requires=['setuptools'],
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
            'youbot_ros2_controller = youbot_webots_controller.youbot_ros2_controller:main',
        ],
    },
)