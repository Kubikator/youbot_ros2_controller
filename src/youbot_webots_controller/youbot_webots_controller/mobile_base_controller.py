#!/usr/bin/env python3

"""
ROS2 Node для управления мобильной платформой KUKA YouBot
Только команды скорости для базы
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


class MobileBaseController(Node):
    def __init__(self):
        super().__init__('youbot_mobile_base_controller')
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/mobile_base/status', 10)
        
        # Subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.command_subscriber = self.create_subscription(
            String, '/mobile_base/commands', self.command_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # Variables
        self.laser_data = None
        self.current_pose = None
        self.current_mode = 'manual'  # manual, autonomous, follow_wall, go_to_point
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        
        # Control parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.safe_distance = 0.5
        
        # Current velocity command
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.get_logger().info('YouBot Mobile Base Controller started')
        self.publish_status('initialized')

    def laser_callback(self, msg):
        """Callback для данных лазерного сканера"""
        self.laser_data = msg
        
    def odom_callback(self, msg):
        """Callback для данных одометрии"""
        self.current_pose = msg.pose.pose

    def command_callback(self, msg):
        """Callback для команд управления"""
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'forward':
            self.move_forward()
        elif command == 'backward':
            self.move_backward()
        elif command == 'left':
            self.turn_left()
        elif command == 'right':
            self.turn_right()
        elif command == 'stop':
            self.stop()
        elif command == 'autonomous':
            self.set_mode('autonomous')
        elif command == 'manual':
            self.set_mode('manual')
        elif command == 'follow_wall':
            self.set_mode('follow_wall')
        elif command == 'go_to_point':
            self.set_mode('go_to_point')
        elif command.startswith('target'):
            # Формат: "target x y theta"
            parts = command.split()
            if len(parts) >= 3:
                try:
                    x, y = float(parts[1]), float(parts[2])
                    theta = float(parts[3]) if len(parts) > 3 else 0.0
                    self.set_target(x, y, theta)
                except ValueError:
                    self.get_logger().warning('Invalid target format. Use: target x y [theta]')

    def control_loop(self):
        """Основной цикл управления"""
        if self.current_mode == 'autonomous':
            self.autonomous_navigation()
        elif self.current_mode == 'follow_wall':
            self.wall_following()
        elif self.current_mode == 'go_to_point':
            self.go_to_point()
        
        # Публикуем текущие команды скорости
        self.publish_velocity(self.current_linear, self.current_angular)

    def move_forward(self, speed=None):
        """Движение вперед"""
        if speed is None:
            speed = self.linear_speed
        self.current_linear = speed
        self.current_angular = 0.0
        self.get_logger().info(f'Moving forward: {speed} m/s')
        self.publish_status(f'moving_forward_{speed}')

    def move_backward(self, speed=None):
        """Движение назад"""
        if speed is None:
            speed = self.linear_speed
        self.current_linear = -speed
        self.current_angular = 0.0
        self.get_logger().info(f'Moving backward: {speed} m/s')
        self.publish_status(f'moving_backward_{speed}')

    def turn_left(self, angular_speed=None):
        """Поворот влево"""
        if angular_speed is None:
            angular_speed = self.angular_speed
        self.current_linear = 0.0
        self.current_angular = angular_speed
        self.get_logger().info(f'Turning left: {angular_speed} rad/s')
        self.publish_status(f'turning_left_{angular_speed}')

    def turn_right(self, angular_speed=None):
        """Поворот вправо"""
        if angular_speed is None:
            angular_speed = self.angular_speed
        self.current_linear = 0.0
        self.current_angular = -angular_speed
        self.get_logger().info(f'Turning right: {angular_speed} rad/s')
        self.publish_status(f'turning_right_{angular_speed}')

    def stop(self):
        """Остановка робота"""
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.get_logger().info('Stopping robot')
        self.publish_status('stopped')

    def set_velocity(self, linear, angular):
        """Прямая установка скорости"""
        self.current_linear = linear
        self.current_angular = angular

    def publish_velocity(self, linear, angular):
        """Отправка команд скорости"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)

    def publish_status(self, status):
        """Публикация статуса"""
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)

    def set_mode(self, mode):
        """Установка режима управления"""
        valid_modes = ['manual', 'autonomous', 'follow_wall', 'go_to_point']
        if mode in valid_modes:
            self.current_mode = mode
            self.get_logger().info(f'Mode changed to: {mode}')
            self.publish_status(f'mode_{mode}')
        else:
            self.get_logger().warning(f'Invalid mode: {mode}')

    def set_target(self, x, y, theta=0.0):
        """Установка целевой точки"""
        self.target_x = x
        self.target_y = y
        self.target_theta = theta
        self.get_logger().info(f'Target set: x={x}, y={y}, theta={theta}')
        self.publish_status(f'target_set_{x}_{y}_{theta}')

    def autonomous_navigation(self):
        """Автономная навигация с избеганием препятствий"""
        if self.laser_data is None:
            return
            
        front_distance = self.get_laser_distance(0)
        left_distance = self.get_laser_distance(90)
        right_distance = self.get_laser_distance(-90)
        
        if front_distance > self.safe_distance:
            self.set_velocity(self.linear_speed, 0.0)
        elif left_distance > right_distance:
            self.set_velocity(0.0, self.angular_speed)
        else:
            self.set_velocity(0.0, -self.angular_speed)

    def wall_following(self):
        """Следование вдоль стены"""
        if self.laser_data is None:
            return
            
        front_distance = self.get_laser_distance(0)
        right_distance = self.get_laser_distance(-90)
        desired_wall_distance = 0.3
        
        if front_distance < self.safe_distance:
            self.set_velocity(0.0, self.angular_speed)
        elif right_distance > desired_wall_distance + 0.1:
            self.set_velocity(self.linear_speed, -self.angular_speed * 0.5)
        elif right_distance < desired_wall_distance - 0.1:
            self.set_velocity(self.linear_speed, self.angular_speed * 0.5)
        else:
            self.set_velocity(self.linear_speed, 0.0)

    def go_to_point(self):
        """Движение к заданной точке"""
        if self.current_pose is None:
            return
            
        dx = self.target_x - self.current_pose.position.x
        dy = self.target_y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            self.set_velocity(0.0, 0.0)
            self.publish_status('target_reached')
            return
            
        target_angle = math.atan2(dy, dx)
        current_angle = self.get_yaw_from_quaternion()
        angle_error = self.normalize_angle(target_angle - current_angle)
        
        if abs(angle_error) > 0.1:
            angular_vel = 0.5 * angle_error
            self.set_velocity(0.0, angular_vel)
        else:
            linear_vel = min(0.3, distance)
            self.set_velocity(linear_vel, 0.0)

    def get_laser_distance(self, angle_deg):
        """Получить расстояние от лазера в заданном направлении"""
        if self.laser_data is None:
            return float('inf')
            
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - self.laser_data.angle_min) / self.laser_data.angle_increment)
        
        if 0 <= index < len(self.laser_data.ranges):
            distance = self.laser_data.ranges[index]
            return distance if not math.isinf(distance) else 10.0
        return 10.0

    def get_yaw_from_quaternion(self):
        """Извлечение угла yaw из кватерниона"""
        if self.current_pose is None:
            return 0.0
            
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    
    mobile_base_controller = MobileBaseController()
    
    try:
        rclpy.spin(mobile_base_controller)
    except KeyboardInterrupt:
        mobile_base_controller.get_logger().info('Shutting down Mobile Base Controller...')
        mobile_base_controller.stop()
    finally:
        mobile_base_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()