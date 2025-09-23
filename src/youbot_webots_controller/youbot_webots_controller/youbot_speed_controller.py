#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import os

# Проверяем и устанавливаем WEBOTS_HOME если не установлен
if 'WEBOTS_HOME' not in os.environ:
    webots_paths = ['/usr/local/webots', '/opt/webots', '/Applications/Webots.app']
    for path in webots_paths:
        if os.path.exists(path):
            os.environ['WEBOTS_HOME'] = path
            break
    else:
        print("ERROR: WEBOTS_HOME not set and Webots installation not found in standard paths")
        print("Please set WEBOTS_HOME environment variable to your Webots installation path")
        sys.exit(1)

# Добавляем путь к Webots Python API
sys.path.append(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'python'))

try:
    from controller import Robot, Motor
except ImportError as e:
    print(f"Failed to import Webots controller: {e}")
    print(f"WEBOTS_HOME is set to: {os.environ.get('WEBOTS_HOME')}")
    print(f"Python path: {sys.path}")
    sys.exit(1)

class YoubotSpeedController(Node):
    def __init__(self):
        super().__init__('youbot_speed_controller')
        
        self.get_logger().info('Starting YouBot ROS 2 Controller...')
        self.get_logger().info(f'WEBOTS_HOME: {os.environ.get("WEBOTS_HOME")}')
        
        try:
            # Инициализация Webots робота
            self.robot = Robot()
            self.timestep = int(self.robot.getBasicTimeStep())
            
            self.get_logger().info('Webots Robot initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Webots robot: {e}')
            raise
        
        # Инициализация моторов колес
        self.wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
        self.wheels = []
        
        for name in self.wheel_names:
            try:
                wheel = self.robot.getDevice(name)
                wheel.setPosition(float('inf'))  # Режим управления скоростью
                wheel.setVelocity(0.0)
                self.wheels.append(wheel)
                self.get_logger().info(f'Initialized wheel: {name}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize wheel {name}: {e}')
                # Продолжаем работу даже если некоторые колеса не инициализировались
        
        # Подписчик для управления скоростью
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('YouBot ROS 2 Controller initialized successfully')
        self.get_logger().info('Waiting for Webots simulation to start...')

    def cmd_vel_callback(self, msg):
        try:
            # Преобразуем Twist в скорости колес для омни-платформы
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z
            
            # Параметры youBot
            wheel_radius = 0.05
            lx = 0.228  # половина расстояния между колесами по X
            ly = 0.158  # половина расстояния между колесами по Y
            
            # Кинематика для 4-х колесной омни-платформы
            wheel_speeds = [
                (linear_x - linear_y - (lx + ly) * angular_z) / wheel_radius,
                (linear_x + linear_y + (lx + ly) * angular_z) / wheel_radius,
                (linear_x + linear_y - (lx + ly) * angular_z) / wheel_radius,
                (linear_x - linear_y + (lx + ly) * angular_z) / wheel_radius
            ]
            
            # Ограничение максимальной скорости
            max_speed = 5.0
            for i in range(len(wheel_speeds)):
                if abs(wheel_speeds[i]) > max_speed:
                    wheel_speeds[i] = max_speed if wheel_speeds[i] > 0 else -max_speed
            
            # Устанавливаем скорости колес
            for i, wheel in enumerate(self.wheels):
                if i < len(wheel_speeds) and wheel is not None:
                    wheel.setVelocity(wheel_speeds[i])
            
            self.get_logger().info(f'Cmd_vel: linear=({linear_x:.2f}, {linear_y:.2f}), angular={angular_z:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {e}')

    def run(self):
        self.get_logger().info('Starting YouBot control loop...')
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
            self.get_logger().info('Controller stopped by user')
        finally:
            # Останавливаем колеса при выходе
            for wheel in self.wheels:
                if wheel is not None:
                    wheel.setVelocity(0.0)
            self.get_logger().info('YouBot controller shutdown')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = YoubotSpeedController()
        controller.run()
    except Exception as e:
        print(f"Failed to start YouBot controller: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()