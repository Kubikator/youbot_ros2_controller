#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
from controller import Robot, Motor
# import sys
# import os

# # Проверяем и устанавливаем WEBOTS_HOME если не установлен
# if 'WEBOTS_HOME' not in os.environ:
#     webots_paths = ['/usr/local/webots', '/opt/webots', '/Applications/Webots.app']
#     for path in webots_paths:
#         if os.path.exists(path):
#             os.environ['WEBOTS_HOME'] = path
#             break
#     else:
#         print("ERROR: WEBOTS_HOME not set and Webots installation not found in standard paths")
#         print("Please set WEBOTS_HOME environment variable to your Webots installation path")
#         sys.exit(1)

# # Добавляем путь к Webots Python API
# sys.path.append(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'python'))

# try:
#     from controller import Robot, Motor
# except ImportError as e:
#     print(f"Failed to import Webots controller: {e}")
#     print(f"WEBOTS_HOME is set to: {os.environ.get('WEBOTS_HOME')}")
#     print(f"Python path: {sys.path}")
#     sys.exit(1)

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
                raise
        
        # Подписчик для получения скоростей колес от ноды кинематики
        self.wheel_speeds_sub = self.create_subscription(
            Float64MultiArray,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )

        # Инициализация сочленений манипулятора YouBot
        self.arm_joint_names = [
            'arm1', 
            'arm2',
            'arm3',
            'arm4',
            'arm5' 
        ]
        
        # Инициализация моторов манипулятора
        self.arm_motors = []
        
        for joint_name in self.arm_joint_names:
            try:
                motor = self.robot.getDevice(joint_name)
                motor.setPosition(0.0)  # Начальная позиция
                self.arm_motors.append(motor)
                self.get_logger().info(f'Initialized arm motor: {joint_name}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize arm motor {joint_name}: {e}')
                raise
        
        # Инициализация датчиков положения
        self.position_sensors = []
        for joint_name in self.arm_joint_names:
            try:
                sensor_name = joint_name + 'sensor'
                sensor = self.robot.getDevice(sensor_name)
                if sensor:
                    sensor.enable(self.timestep)
                    self.position_sensors.append(sensor)
                    self.get_logger().info(f'Initialized position sensor: {sensor_name}')
                else:
                    self.get_logger().warning(f'Position sensor {sensor_name} not found')
                    self.position_sensors.append(None)
            except Exception as e:
                self.get_logger().error(f'Failed to initialize sensor {joint_name}: {e}')
                self.position_sensors.append(None)
        
        
        # Подписчик для получения целевых позиций манипулятора
        self.arm_target_sub = self.create_subscription(
            Float64MultiArray,
            'arm_target_positions',
            self.arm_target_callback,
            10
        )
        
        # Публикатор для состояний суставов
        self.joint_state_pub = self.create_publisher(
            JointState,
            'arm_joint_states',
            10
        )
        
        # Таймер для обновления состояний суставов
        self.create_timer(0.1, self.update_joint_states)  # 10 Hz

        # Инициализация устройств схвата
        self.finger_motors = {}
        self.finger_sensors = {}
        
        # Левый палец
        try:
            left_motor = self.robot.getDevice("finger::left")
            left_motor.setVelocity(0.03)
            self.finger_motors['left'] = left_motor
            self.get_logger().info('Initialized left finger motor')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize left finger motor: {e}')
            raise
        
        # Правый палец
        try:
            right_motor = self.robot.getDevice("finger::right")
            right_motor.setVelocity(0.03)  # Скорость как в оригинальном коде
            self.finger_motors['right'] = right_motor
            self.get_logger().info('Initialized right finger motor')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize right finger motor: {e}')
            raise
        
        # Датчики положения пальцев
        try:
            left_sensor = self.robot.getDevice("finger::leftsensor")
            left_sensor.enable(self.timestep)
            self.finger_sensors['left'] = left_sensor
            self.get_logger().info('Initialized left finger sensor')
        except Exception as e:
            self.get_logger().warning(f'Failed to initialize left finger sensor: {e}')
            self.finger_sensors['left'] = None
        
        try:
            right_sensor = self.robot.getDevice("finger::rightsensor")
            right_sensor.enable(self.timestep)
            self.finger_sensors['right'] = right_sensor
            self.get_logger().info('Initialized right finger sensor')
        except Exception as e:
            self.get_logger().warning(f'Failed to initialize right finger sensor: {e}')
            self.finger_sensors['right'] = None

        self.position_sub = self.create_subscription(
            Float64,
            'gripper_position',
            self.gripper_position_callback,
            10
        )

        self.gripper_state_pub = self.create_publisher(
            Float64,
            'gripper_current_position',
            10
        )

        # Таймер для публикации состояния
        self.create_timer(0.1, self.publish_gripper_state)  # 10 Hz
        
        self.get_logger().info('Webots Ros2 Bridge initialized successfully')

    def wheel_speeds_callback(self, msg):
        try:
            # Устанавливаем скорости колес полученные от ноды кинематики
            if len(msg.data) == len(self.wheels):
                for i, wheel in enumerate(self.wheels):
                    wheel.setVelocity(msg.data[i])
            else:
                self.get_logger().warn(f'Received {len(msg.data)} speeds, expected {len(self.wheels)}')
                
        except Exception as e:
            self.get_logger().error(f'Error in wheel_speeds_callback: {e}')

    def arm_target_callback(self, msg):
        try:
            if len(msg.data) == len(self.arm_motors):
                for i, motor in enumerate(self.arm_motors):
                    motor.setPosition(msg.data[i])
            else:
                self.get_logger().warn(f'Received {len(msg.data)} positions, expected {len(self.arm_motors)}')
                
        except Exception as e:
            self.get_logger().error(f'Error in arm_target_callback: {e}')

    def update_joint_states(self):
        try:
            # Чтение текущих позиций из датчиков
            current_positions = []
            for sensor in self.position_sensors:
                if sensor is not None:
                    current_positions.append(sensor.getValue())
                else:
                    current_positions.append(0.0)  # Заглушка если датчик отсутствует
            
            # Публикация состояния суставов
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.arm_joint_names
            joint_state_msg.position = current_positions
            
            self.joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error updating joint states: {e}')
            
        except Exception as e:
            self.get_logger().error(f'Error in set_gap: {e}')

    def gripper_position_callback(self, msg):
        position = msg.data
        self.get_logger().info(f'Setting gripper position: {position}')
        try:
            for motor in self.finger_motors.values():
                motor.setPosition(position)
            
            self.get_logger().info(f'Set gripper position: {position}')
            
        except Exception as e:
            self.get_logger().error(f'Error in set_position: {e}')

    
    def publish_gripper_state(self):
        try:
            positions = {}
            for side, sensor in self.finger_sensors.items():
                if sensor is not None:
                    positions[side] = sensor.getValue()
                else:
                    positions[side] = None

            if positions['left'] is not None:
                
                state_msg = Float64()
                state_msg.data = positions['left']
                self.gripper_state_pub.publish(state_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing gripper state: {e}')

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
            # Останавливаем колеса при выходе
            for wheel in self.wheels:
                if wheel is not None:
                    wheel.setVelocity(0.0)
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