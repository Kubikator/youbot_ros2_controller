#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class PlatformKinematicsSolver(Node):
    def __init__(self):
        super().__init__('platform_kinematics_solver')
        
        self.get_logger().info('Starting Platform Kinematics Solver...')
        
        # Параметры youBot
        self.wheel_radius = 0.05
        self.lx = 0.228  # половина расстояния между колесами по X
        self.ly = 0.158  # половина расстояния между колесами по Y
        
        # Максимальная скорость колес
        self.max_wheel_speed = 5.0
        
        # Подписчик для команд управления
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Публикатор для скоростей колес
        self.wheel_speeds_pub = self.create_publisher(
            Float64MultiArray,
            'wheel_speeds',
            10
        )
        
        self.get_logger().info('Platform Kinematics Solver initialized successfully')

    def cmd_vel_callback(self, msg):
        try:
            # Получаем команды управления
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z
            
            # Вычисляем скорости колес используя кинематику омни-платформы
            wheel_speeds = self.calculate_wheel_speeds(linear_x, linear_y, angular_z)
            
            # Ограничиваем максимальную скорость
            wheel_speeds = self.limit_wheel_speeds(wheel_speeds)
            
            # Публикуем скорости колес
            wheel_speeds_msg = Float64MultiArray()
            wheel_speeds_msg.data = wheel_speeds
            self.wheel_speeds_pub.publish(wheel_speeds_msg)
            
            self.get_logger().info(
                f'Cmd_vel: linear=({linear_x:.2f}, {linear_y:.2f}), angular={angular_z:.2f} -> '
                f'Wheel speeds: [{wheel_speeds[0]:.2f}, {wheel_speeds[1]:.2f}, {wheel_speeds[2]:.2f}, {wheel_speeds[3]:.2f}]'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {e}')

    def calculate_wheel_speeds(self, linear_x, linear_y, angular_z):
        # Кинематика для 4-х колесной омни-платформы
        wheel_speeds = [
            (linear_x - linear_y - (self.lx + self.ly) * angular_z) / self.wheel_radius,
            (linear_x + linear_y + (self.lx + self.ly) * angular_z) / self.wheel_radius,
            (linear_x + linear_y - (self.lx + self.ly) * angular_z) / self.wheel_radius,
            (linear_x - linear_y + (self.lx + self.ly) * angular_z) / self.wheel_radius
        ]
        return wheel_speeds

    def limit_wheel_speeds(self, wheel_speeds):
        limited_speeds = []
        for speed in wheel_speeds:
            if abs(speed) > self.max_wheel_speed:
                limited_speeds.append(self.max_wheel_speed if speed > 0 else -self.max_wheel_speed)
            else:
                limited_speeds.append(speed)
        return limited_speeds

def main(args=None):
    rclpy.init(args=args)
    
    try:
        kinematics_calculator = PlatformKinematicsSolver()
        rclpy.spin(kinematics_calculator)
    except Exception as e:
        print(f"Failed to start Platform Kinematics Solver: {e}")
    finally:
        kinematics_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()