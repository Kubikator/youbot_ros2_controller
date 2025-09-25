import rclpy
from std_msgs.msg import Float64MultiArray

class MobilePlatform:
    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        
        self.node.get_logger().info('Initializing Mobile Platform...')
        
        # Инициализация моторов колес
        self.wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
        self.wheels = []
        
        for name in self.wheel_names:
            try:
                wheel = self.robot.getDevice(name)
                wheel.setPosition(float('inf'))  # Режим управления скоростью
                wheel.setVelocity(0.0)
                self.wheels.append(wheel)
                self.node.get_logger().info(f'Initialized wheel: {name}')
            except Exception as e:
                self.node.get_logger().error(f'Failed to initialize wheel {name}: {e}')
                raise
        
        # Подписчик для получения скоростей колес от ноды кинематики
        self.wheel_speeds_sub = self.node.create_subscription(
            Float64MultiArray,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        
        self.node.get_logger().info('Mobile Platform initialized successfully')

    def wheel_speeds_callback(self, msg):
        try:
            # Устанавливаем скорости колес полученные от ноды кинематики
            if len(msg.data) == len(self.wheels):
                for i, wheel in enumerate(self.wheels):
                    wheel.setVelocity(msg.data[i])
            else:
                self.node.get_logger().warn(f'Received {len(msg.data)} speeds, expected {len(self.wheels)}')
                
        except Exception as e:
            self.node.get_logger().error(f'Error in wheel_speeds_callback: {e}')

    def shutdown(self):
        # Останавливаем колеса при выходе
        for wheel in self.wheels:
            if wheel is not None:
                wheel.setVelocity(0.0)
        self.node.get_logger().info('Mobile Platform shutdown')
        