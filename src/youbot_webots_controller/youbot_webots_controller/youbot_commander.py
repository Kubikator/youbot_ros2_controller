#!/usr/bin/env python3

"""
ROS2 Node для отправки команд мобильной платформе и манипулятору
Интерфейс командной строки и программный интерфейс
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import sys
import time


class YouBotCommander(Node):
    def __init__(self):
        super().__init__('youbot_commander')
        
        # Publishers для отправки команд
        self.mobile_command_publisher = self.create_publisher(String, '/mobile_base/commands', 10)
        self.arm_command_publisher = self.create_publisher(String, '/arm/commands', 10)
        self.arm_position_publisher = self.create_publisher(Float64MultiArray, '/arm/joint_positions', 10)
        
        # Subscribers для получения статуса
        self.mobile_status_subscriber = self.create_subscription(
            String, '/mobile_base/status', self.mobile_status_callback, 10)
        self.arm_status_subscriber = self.create_subscription(
            String, '/arm/status', self.arm_status_callback, 10)
        
        self.mobile_status = 'unknown'
        self.arm_status = 'unknown'
        
        self.get_logger().info('YouBot Commander started')

    def mobile_status_callback(self, msg):
        """Callback для статуса мобильной базы"""
        self.mobile_status = msg.data
        
    def arm_status_callback(self, msg):
        """Callback для статуса манипулятора"""
        self.arm_status = msg.data

    def send_mobile_command(self, command):
        """Отправка команды мобильной базе"""
        msg = String()
        msg.data = command
        self.mobile_command_publisher.publish(msg)
        self.get_logger().info(f'Sent mobile command: {command}')

    def send_arm_command(self, command):
        """Отправка команды манипулятору"""
        msg = String()
        msg.data = command
        self.arm_command_publisher.publish(msg)
        self.get_logger().info(f'Sent arm command: {command}')

    def send_arm_positions(self, positions):
        """Отправка позиций суставов манипулятора"""
        if len(positions) == 5:
            msg = Float64MultiArray()
            msg.data = positions
            self.arm_position_publisher.publish(msg)
            self.get_logger().info(f'Sent arm positions: {positions}')
        else:
            self.get_logger().warning('Arm positions must contain exactly 5 values')

    def get_status(self):
        """Получение текущего статуса системы"""
        return {
            'mobile_base': self.mobile_status,
            'arm': self.arm_status
        }

    def print_help(self):
        """Вывод справки по командам"""
        help_text = """
        ==================== YOUBOT COMMANDER ====================
        
        МОБИЛЬНАЯ БАЗА:
        forward, backward, left, right, stop
        autonomous, manual, follow_wall, go_to_point
        target <x> <y> [theta] - установить цель для go_to_point
        
        МАНИПУЛЯТОР:
        home, candle, extended, folded, grasp_front, grasp_side
        joint <index> <angle> - установить угол сустава (index: 0-4)
        current - показать текущие позиции суставов
        
        СИСТЕМНЫЕ:
        status - показать статус всех компонентов
        help - показать эту справку
        quit - выход
        ========================================================
        """
        print(help_text)

    def interactive_mode(self):
        """Интерактивный режим командной строки"""
        self.print_help()
        
        while True:
            try:
                command = input("youbot> ").strip()
                
                if not command:
                    continue
                    
                if command.lower() in ['quit', 'exit', 'q']:
                    break
                elif command.lower() == 'help':
                    self.print_help()
                elif command.lower() == 'status':
                    status = self.get_status()
                    print(f"Mobile Base: {status['mobile_base']}")
                    print(f"Arm: {status['arm']}")
                else:
                    self.process_command(command)
                    
                # Небольшая задержка для обработки ROS2 сообщений
                rclpy.spin_once(self, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
        
        print("Exiting YouBot Commander...")

    def process_command(self, command):
        """Обработка команды и определение, куда её отправить"""
        command = command.lower().strip()
        
        # Команды мобильной базы
        mobile_commands = [
            'forward', 'backward', 'left', 'right', 'stop',
            'autonomous', 'manual', 'follow_wall', 'go_to_point'
        ]
        
        # Команды манипулятора
        arm_commands = [
            'home', 'candle', 'extended', 'folded', 'grasp_front', 'grasp_side',
            'current'
        ]
        
        if command in mobile_commands:
            self.send_mobile_command(command)
        elif command in arm_commands:
            self.send_arm_command(command)
        elif command.startswith('target'):
            self.send_mobile_command(command)
        elif command.startswith('joint'):
            self.send_arm_command(command)
        elif command.startswith('positions'):
            # Формат: "positions 0.0 1.0 -1.0 0.5 0.0"
            parts = command.split()[1:]
            if len(parts) == 5:
                try:
                    positions = [float(p) for p in parts]
                    self.send_arm_positions(positions)
                except ValueError:
                    print("Invalid position values. Use: positions <5 float values>")
            else:
                print("Invalid format. Use: positions <5 float values>")
        else:
            print(f"Unknown command: {command}. Type 'help' for available commands.")


def main(args=None):
    rclpy.init(args=args)
    
    commander = YouBotCommander()
    
    # Проверяем аргументы командной строки
    if len(sys.argv) > 1:
        # Неинтерактивный режим - выполняем команду из аргументов
        command = ' '.join(sys.argv[1:])
        commander.process_command(command)
        
        # Ждем немного для обработки команды
        time.sleep(0.5)
        rclpy.spin_once(commander, timeout_sec=0.1)
        
    else:
        # Интерактивный режим
        try:
            commander.interactive_mode()
        except KeyboardInterrupt:
            pass
    
    commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()