#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from youbot_webots_controller.webots_ros_bridge.mobile_platform import MobilePlatform
from youbot_webots_controller.webots_ros_bridge.manipulator import Manipulator
from youbot_webots_controller.webots_ros_bridge.gripper import Gripper
from youbot_webots_controller.webots_ros_bridge.lidar import Lidar
from youbot_webots_controller.webots_ros_bridge.camera import WebotsCamera
from controller import Robot
import numpy as np

class WebotsRosBridge(Node):
    def __init__(self):
        super().__init__('webots_bridge')
        
        self.get_logger().info('Starting Webots Bridge...')
        
        try:
            # Инициализация Webots робота
            self.robot = Robot()
            self.timestep = int(self.robot.getBasicTimeStep())
            self.get_logger().info('Webots Robot initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Webots robot: {e}')
            raise
        
        # Инициализация компонентов
        self.mobile_platform = MobilePlatform(self, self.robot)
        self.manipulator = Manipulator(self, self.robot, self.timestep)
        self.gripper = Gripper(self, self.robot, self.timestep)
        self.lidar = Lidar(self, self.robot, self.timestep)
        self.left_camera = WebotsCamera(
            node=self,
            camera_name='camera_left',  # Имя камеры в Webots
            tf_prefix='left',           # Префикс для топиков и TF
            camera_offset_x=0.3,
            camera_offset_y=0.1,        # Смещение влево
            camera_offset_z=0.1,
            rotation_z=-10.0             # Поворот на 10 градусов
        )
        
        # Правая камера  
        self.right_camera = WebotsCamera(
            node=self,
            camera_name='camera_right',  # Имя камеры в Webots
            tf_prefix='right',           # Префикс для топиков и TF
            camera_offset_x=0.3,
            camera_offset_y=-0.1,        # Смещение вправо
            camera_offset_z=0.1,
            rotation_z=10.0             # Поворот на -10 градусов
        )    
        
        self.get_logger().info('Webots Ros2 Bridge initialized successfully')

    def run(self):
        self.get_logger().info('Starting Webots Bridge control loop...')
        try:
            while rclpy.ok():
                try:
                    if self.robot.step(self.timestep) == -1:
                        self.get_logger().warn('Webots simulation ended')
                        break
                    rclpy.spin_once(self, timeout_sec=0.001)
                except Exception as e:
                    self.get_logger().error(f'Error in control loop: {e}')
                    break
        except KeyboardInterrupt:
            self.get_logger().info('Webots Bridge stopped by user')
        finally:
            # Останавливаем компоненты при выходе
            self.mobile_platform.shutdown()
            self.left_camera.shutdown()
            self.right_camera.shutdown()
            self.lidar.shutdown()
            self.get_logger().info('Webots Bridge shutdown')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = WebotsRosBridge()
        bridge.run()
    except Exception as e:
        print(f"Failed to start Webots Bridge: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()