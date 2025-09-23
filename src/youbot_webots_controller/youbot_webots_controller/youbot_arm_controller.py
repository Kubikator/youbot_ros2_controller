#!/usr/bin/env python3

"""
ROS2 Node для управления манипулятором KUKA YouBot
Только управление углами звеньев
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
import math
import time


class ArmController(Node):
    def __init__(self):
        super().__init__('youbot_arm_controller')
        
        # Publishers
        self.arm_publisher = self.create_publisher(JointState, '/arm_joint_commands', 10)
        self.status_publisher = self.create_publisher(String, '/arm/status', 10)
        
        # Subscribers
        self.joint_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.command_subscriber = self.create_subscription(
            String, '/arm/commands', self.command_callback, 10)
        self.position_subscriber = self.create_subscription(
            Float64MultiArray, '/arm/joint_positions', self.position_callback, 10)
            
        # Timer для регулярной публикации команд
        self.command_timer = self.create_timer(0.1, self.publish_joint_commands)
        
        # Joint names для YouBot
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
        
        # Current joint positions
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_states = None
        
        # Predefined positions
        self.predefined_positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            'candle': [0.0, 0.0, -1.57, 0.0, 0.0],
            'extended': [0.0, -1.57, 0.0, -1.57, 0.0],
            'folded': [2.94, 1.57, -2.55, 1.78, 2.94],
            'grasp_front': [0.0, -1.0, 1.0, -1.5, 0.0],
            'grasp_side': [1.57, -1.0, 1.0, -1.5, 0.0]
        }
        
        # Joint limits (радианы)
        self.joint_limits = {
            'arm_joint_1': (-2.94, 2.94),
            'arm_joint_2': (-1.57, 1.13),
            'arm_joint_3': (-2.55, 2.55),
            'arm_joint_4': (-1.78, 1.78),
            'arm_joint_5': (-2.94, 2.94)
        }
        
        self.get_logger().info('YouBot Arm Controller started')
        self.get_logger().info(f'Available positions: {list(self.predefined_positions.keys())}')
        self.publish_status('initialized')

    def joint_callback(self, msg):
        """Callback для получения текущего состояния суставов"""
        self.current_joint_states = msg
        
        # Обновляем текущие позиции суставов манипулятора
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                joint_index = msg.name.index(name)
                if joint_index < len(msg.position):
                    self.current_joint_positions[i] = msg.position[joint_index]

    def command_callback(self, msg):
        """Callback для команд управления манипулятором"""
        command = msg.data.lower()
        self.get_logger().info(f'Received arm command: {command}')
        
        if command in self.predefined_positions:
            self.move_to_position(command)
        elif command == 'current':
            self.print_current_positions()
        elif command.startswith('joint'):
            # Формат: "joint 0 1.57" (сустав 0 в позицию 1.57 радиан)
            parts = command.split()
            if len(parts) == 3:
                try:
                    joint_index = int(parts[1])
                    angle = float(parts[2])
                    self.set_joint_angle(joint_index, angle)
                except (ValueError, IndexError):
                    self.get_logger().warning('Invalid joint command format. Use: joint <index> <angle>')
        else:
            self.get_logger().warning(f'Unknown arm command: {command}')

    def position_callback(self, msg):
        """Callback для прямой установки позиций суставов"""
        if len(msg.data) == 5:
            self.target_joint_positions = list(msg.data)
            self.get_logger().info(f'Received joint positions: {self.target_joint_positions}')
            self.publish_status('position_received')
        else:
            self.get_logger().warning('Invalid joint position array. Expected 5 values.')

    def move_to_position(self, position_name):
        """Перемещение в предопределенную позицию"""
        if position_name in self.predefined_positions:
            self.target_joint_positions = self.predefined_positions[position_name].copy()
            self.get_logger().info(f'Moving to position: {position_name}')
            self.publish_status(f'moving_to_{position_name}')
        else:
            self.get_logger().warning(f'Unknown position: {position_name}')

    def set_joint_angle(self, joint_index, angle):
        """Установка угла конкретного сустава"""
        if 0 <= joint_index < 5:
            # Проверяем ограничения
            joint_name = self.joint_names[joint_index]
            min_limit, max_limit = self.joint_limits[joint_name]
            
            if min_limit <= angle <= max_limit:
                self.target_joint_positions[joint_index] = angle
                self.get_logger().info(f'Set {joint_name} to {angle} rad')
                self.publish_status(f'joint_{joint_index}_set_{angle}')
            else:
                self.get_logger().warning(f'Angle {angle} out of limits for {joint_name}: [{min_limit}, {max_limit}]')
        else:
            self.get_logger().warning(f'Invalid joint index: {joint_index}. Valid range: 0-4')

    def set_all_joints(self, joint_positions):
        """Установка всех углов суставов одновременно"""
        if len(joint_positions) == 5:
            valid_positions = []
            
            # Проверяем все ограничения
            for i, angle in enumerate(joint_positions):
                joint_name = self.joint_names[i]
                min_limit, max_limit = self.joint_limits[joint_name]
                
                if min_limit <= angle <= max_limit:
                    valid_positions.append(angle)
                else:
                    self.get_logger().warning(f'Angle {angle} out of limits for {joint_name}')
                    valid_positions.append(max(min_limit, min(angle, max_limit)))  # Ограничиваем значение
            
            self.target_joint_positions = valid_positions
            self.get_logger().info(f'Set all joints to: {self.target_joint_positions}')
            self.publish_status('all_joints_set')
        else:
            self.get_logger().warning('Invalid number of joint positions. Expected 5 values.')

    def publish_joint_commands(self):
        """Публикация команд для суставов"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.target_joint_positions
        
        self.arm_publisher.publish(joint_msg)

    def publish_status(self, status):
        """Публикация статуса манипулятора"""
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)

    def print_current_positions(self):
        """Вывод текущих позиций суставов"""
        self.get_logger().info('Current joint positions:')
        for i, (name, pos) in enumerate(zip(self.joint_names, self.current_joint_positions)):
            self.get_logger().info(f'  {name} (joint {i}): {pos:.3f} rad ({math.degrees(pos):.1f}°)')

    def get_current_position_as_list(self):
        """Возвращает текущие позиции как список"""
        return self.current_joint_positions.copy()

    def interpolate_to_position(self, target_positions, duration_seconds=3.0, steps=30):
        """Плавное движение к позиции за заданное время"""
        if len(target_positions) != 5:
            self.get_logger().warning('Invalid target positions length')
            return
            
        start_positions = self.current_joint_positions.copy()
        
        for step in range(steps + 1):
            alpha = step / steps  # От 0 до 1
            
            # Линейная интерполяция для каждого сустава
            interpolated_positions = []
            for start, target in zip(start_positions, target_positions):
                interpolated_pos = start + alpha * (target - start)
                interpolated_positions.append(interpolated_pos)
            
            self.target_joint_positions = interpolated_positions
            
            # Ждем между шагами
            time.sleep(duration_seconds / steps)
            
        self.get_logger().info('Interpolated movement completed')
        self.publish_status('interpolation_completed')

    def emergency_stop(self):
        """Экстренная остановка - фиксация текущих позиций"""
        self.target_joint_positions = self.current_joint_positions.copy()
        self.get_logger().warning('EMERGENCY STOP - Arm positions locked')
        self.publish_status('emergency_stop')

    def add_custom_position(self, name, positions):
        """Добавление пользовательской позиции"""
        if len(positions) == 5:
            self.predefined_positions[name] = positions
            self.get_logger().info(f'Added custom position "{name}": {positions}')
            self.publish_status(f'custom_position_added_{name}')
        else:
            self.get_logger().warning('Custom position must have 5 joint values')


def main(args=None):
    rclpy.init(args=args)
    
    arm_controller = ArmController()
    
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        arm_controller.get_logger().info('Shutting down Arm Controller...')
        arm_controller.emergency_stop()
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()