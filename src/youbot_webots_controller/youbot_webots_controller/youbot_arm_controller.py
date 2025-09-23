#!/usr/bin/env python3
"""
YouBot Arm Controller для Webots - исправленная версия
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, String
import math
import time
from threading import Thread

class YouBotArmController(Node):
    def __init__(self):
        super().__init__('youbot_arm_controller')
        
        self.get_logger().info('Инициализация YouBot Arm Controller...')
        
        # Параметры манипулятора YouBot
        self.arm_joint_names = [
            'arm1',  # Основание (поворот)
            'arm2',  # Плечо (подъем/опускание)
            'arm3',  # Локоть (подъем/опускание)
            'arm4',  # Запястье 1 (поворот)
            'arm5'   # Запястье 2 (наклон)
        ]
        
        # Пределы углов для каждого сустава (в радианах)
        self.joint_limits = {
            'arm1': (-2.95, 2.95),
            'arm2': (-1.13, 1.13),
            'arm3': (-2.07, 2.07),
            'arm4': (-1.57, 1.57),
            'arm5': (-2.07, 2.07)
        }
        
        # Текущие позиции суставов
        self.current_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.is_moving = False
        
        # Webots устройства
        self.motors = []
        self.position_sensors = []
        self.robot = None
        
        # Инициализация Webots
        self.initialize_webots()
        
        # Публикаторы
        self.joint_state_pub = self.create_publisher(JointState, 'arm_joint_states', 10)
        self.arm_status_pub = self.create_publisher(String, 'arm_status', 10)
        
        # Подписчики
        self.create_subscription(Float64MultiArray, 'arm_target_positions', 
                                self.target_positions_callback, 10)
        self.create_subscription(JointTrajectory, 'arm_joint_trajectory',
                                self.joint_trajectory_callback, 10)
        self.create_subscription(String, 'arm_command',
                                self.arm_command_callback, 10)
        
        # Таймеры
        self.create_timer(0.1, self.update_joint_states)  # 10 Hz
        self.create_timer(1.0, self.diagnostic_check)     # 1 Hz для диагностики
        
        # Предустановленные позиции
        self.preset_positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, 0.9, -1.0, 0.5, 0.0],
            'pick_floor': [0.0, 1.0, -1.5, 0.5, 0.0],
            'pick_table': [0.0, 0.5, -0.8, 0.3, 0.0],
            'transport': [0.0, 0.3, -0.5, 0.2, 0.0],
            'test1': [0.5, 0.5, 0.5, 0.5, 0.5],
            'test2': [-0.5, -0.5, -0.5, -0.5, -0.5]
        }
        
        self.get_logger().info('YouBot Arm Controller готов к работе')

    def initialize_webots(self):
        """Инициализация Webots устройств"""
        try:
            from controller import Robot
            self.robot = Robot()
            
            # Получаем тайм-степ
            self.timestep = int(self.robot.getBasicTimeStep())
            
            self.get_logger().info(f'Тайм-степ: {self.timestep}')
            
            # Инициализация моторов
            for joint_name in self.arm_joint_names:
                try:
                    motor = self.robot.getDevice(joint_name)
                    if motor:
                        # Устанавливаем начальную позицию и скорость
                        motor.setPosition(0.0)
                        motor.setVelocity(1.0)  # Увеличиваем скорость для теста
                        self.motors.append(motor)
                        self.get_logger().info(f'Мотор {joint_name} инициализирован')
                    else:
                        self.get_logger().error(f'Мотор {joint_name} не найден!')
                        # Создаем заглушку для отсутствующего мотора
                        self.motors.append(None)
                except Exception as e:
                    self.get_logger().error(f'Ошибка инициализации мотора {joint_name}: {e}')
                    self.motors.append(None)
            
            # Инициализация датчиков положения
            for joint_name in self.arm_joint_names:
                try:
                    sensor_name = joint_name + '_sensor'
                    sensor = self.robot.getDevice(sensor_name)
                    if sensor:
                        sensor.enable(self.timestep)
                        self.position_sensors.append(sensor)
                        self.get_logger().info(f'Датчик {sensor_name} инициализирован')
                    else:
                        self.get_logger().warning(f'Датчик {sensor_name} не найден')
                        self.position_sensors.append(None)
                except Exception as e:
                    self.get_logger().error(f'Ошибка инициализации датчика {joint_name}: {e}')
                    self.position_sensors.append(None)
            
            self.get_logger().info('Webots устройства инициализированы')
            
        except ImportError:
            self.get_logger().error('Webots controller module not available - running in test mode')
        except Exception as e:
            self.get_logger().error(f'Ошибка инициализации Webots: {e}')

    def target_positions_callback(self, msg):
        """Обработка целевых позиций суставов"""
        self.get_logger().info(f'Получены целевые позиции: {msg.data}')
        
        if len(msg.data) == 5:
            # Проверяем и ограничиваем позиции
            self.target_positions = []
            for i, (pos, name) in enumerate(zip(msg.data, self.arm_joint_names)):
                limited_pos = max(min(pos, self.joint_limits[name][1]), self.joint_limits[name][0])
                self.target_positions.append(limited_pos)
                self.get_logger().info(f'Сустав {name}: {pos} -> {limited_pos}')
            
            self.move_to_target()
        else:
            self.get_logger().warn(f'Неверное количество позиций: {len(msg.data)} (должно быть 5)')

    def joint_trajectory_callback(self, msg):
        """Обработка траектории движения"""
        self.get_logger().info(f'Получена траектория для суставов: {msg.joint_names}')
        
        if set(msg.joint_names) == set(self.arm_joint_names):
            for point in msg.points:
                self.target_positions = [
                    max(min(pos, self.joint_limits[name][1]), self.joint_limits[name][0])
                    for pos, name in zip(point.positions, self.arm_joint_names)
                ]
                self.move_to_target()
                time.sleep(point.time_from_start.sec + point.time_from_start.nanosec * 1e-9)
        else:
            self.get_logger().warn('Несовпадение имен суставов в траектории')

    def arm_command_callback(self, msg):
        """Обработка текстовых команд"""
        command = msg.data.lower()
        self.get_logger().info(f'Получена команда: {command}')
        
        if command in self.preset_positions:
            self.target_positions = self.preset_positions[command].copy()
            self.get_logger().info(f'Перемещение в позицию {command}: {self.target_positions}')
            self.move_to_target()
        
        elif command == 'stop':
            self.stop_movement()
            self.get_logger().info('Движение остановлено')
        
        elif command == 'get_status':
            self.publish_arm_status()
            self.get_logger().info('Статус отправлен')
        
        elif command == 'diagnostic':
            self.run_diagnostic()
        
        else:
            self.get_logger().warn(f'Неизвестная команда: {command}')

    def move_to_target(self):
        """Плавное перемещение к целевым позициям"""
        if self.is_moving:
            self.get_logger().info('Движение уже выполняется, ожидание...')
            return
        
        self.is_moving = True
        self.get_logger().info(f'Начало движения к позициям: {self.target_positions}')
        
        def movement_thread():
            try:
                steps = 20  # Уменьшаем количество шагов для более быстрого движения
                
                for step in range(steps):
                    if not self.is_moving:
                        break
                    
                    # Линейная интерполяция
                    alpha = (step + 1) / steps
                    intermediate = [
                        self.current_positions[i] + alpha * (self.target_positions[i] - self.current_positions[i])
                        for i in range(5)
                    ]
                    
                    # Установка позиции моторов
                    for i, motor in enumerate(self.motors):
                        if motor is not None:
                            motor.setPosition(intermediate[i])
                            if step == 0 or step == steps-1:  # Логируем первый и последний шаг
                                self.get_logger().info(f'Мотор {i}: позиция {intermediate[i]}')
                    
                    # Обновление Webots
                    if self.robot:
                        self.robot.step(self.timestep)
                    
                    time.sleep(0.05)  # 20 Hz
                
                self.is_moving = False
                self.get_logger().info('Движение завершено')
                self.publish_arm_status()
                
            except Exception as e:
                self.get_logger().error(f'Ошибка в потоке движения: {e}')
                self.is_moving = False
        
        Thread(target=movement_thread, daemon=True).start()

    def stop_movement(self):
        """Остановка движения"""
        self.is_moving = False
        self.target_positions = self.current_positions.copy()

    def update_joint_states(self):
        """Обновление и публикация состояний суставов"""
        try:
            # Чтение текущих позиций из датчиков
            new_positions = []
            for i, sensor in enumerate(self.position_sensors):
                if sensor is not None:
                    position = sensor.getValue()
                    new_positions.append(position)
                else:
                    new_positions.append(self.current_positions[i])
            
            self.current_positions = new_positions
            
            # Публикация состояния суставов
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.arm_joint_names
            joint_state_msg.position = self.current_positions
            
            self.joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Ошибка обновления состояний суставов: {e}')

    def publish_arm_status(self):
        """Публикация статуса манипулятора"""
        status_msg = String()
        if self.is_moving:
            status_msg.data = "MOVING"
        else:
            tolerance = 0.1  # Увеличиваем допуск для тестирования
            at_target = all(
                abs(current - target) < tolerance
                for current, target in zip(self.current_positions, self.target_positions)
            )
            status_msg.data = "AT_TARGET" if at_target else "READY"
        
        self.arm_status_pub.publish(status_msg)
        self.get_logger().info(f'Статус манипулятора: {status_msg.data}')

    def diagnostic_check(self):
        """Периодическая диагностика"""
        if not self.is_moving:
            # Логируем текущие позиции раз в 10 секунд
            if int(time.time()) % 10 == 0:
                self.get_logger().info(
                    f'Текущие позиции: {[f"{p:.3f}" for p in self.current_positions]}'
                )

    def run_diagnostic(self):
        """Запуск диагностики"""
        self.get_logger().info('=== ДИАГНОСТИКА МАНИПУЛЯТОРА ===')
        
        # Проверка моторов
        for i, motor in enumerate(self.motors):
            status = "OK" if motor is not None else "MISSING"
            self.get_logger().info(f'Мотор {self.arm_joint_names[i]}: {status}')
        
        # Проверка датчиков
        for i, sensor in enumerate(self.position_sensors):
            status = "OK" if sensor is not None else "MISSING"
            if sensor:
                value = sensor.getValue()
                self.get_logger().info(f'Датчик {self.arm_joint_names[i]}: {value:.3f}')
            else:
                self.get_logger().info(f'Датчик {self.arm_joint_names[i]}: {status}')
        
        self.get_logger().info('=== ДИАГНОСТИКА ЗАВЕРШЕНА ===')

def main(args=None):
    rclpy.init(args=args)
    controller = YouBotArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Контроллер остановлен')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()