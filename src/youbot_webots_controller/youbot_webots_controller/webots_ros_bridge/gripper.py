import rclpy
from std_msgs.msg import Float64

class Gripper:
    def __init__(self, node, robot, timestep):
        self.node = node
        self.robot = robot
        self.timestep = timestep
        
        self.node.get_logger().info('Initializing Gripper...')
        
        # Инициализация устройств схвата
        self.finger_motors = {}
        self.finger_sensors = {}
        
        # Левый палец
        try:
            left_motor = self.robot.getDevice("finger::left")
            left_motor.setVelocity(0.03)
            self.finger_motors['left'] = left_motor
            self.node.get_logger().info('Initialized left finger motor')
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize left finger motor: {e}')
            raise
        
        # Правый палец
        try:
            right_motor = self.robot.getDevice("finger::right")
            right_motor.setVelocity(0.03)  # Скорость как в оригинальном коде
            self.finger_motors['right'] = right_motor
            self.node.get_logger().info('Initialized right finger motor')
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize right finger motor: {e}')
            raise
        
        # Датчики положения пальцев
        try:
            left_sensor = self.robot.getDevice("finger::leftsensor")
            left_sensor.enable(self.timestep)
            self.finger_sensors['left'] = left_sensor
            self.node.get_logger().info('Initialized left finger sensor')
        except Exception as e:
            self.node.get_logger().warning(f'Failed to initialize left finger sensor: {e}')
            self.finger_sensors['left'] = None
        
        try:
            right_sensor = self.robot.getDevice("finger::rightsensor")
            right_sensor.enable(self.timestep)
            self.finger_sensors['right'] = right_sensor
            self.node.get_logger().info('Initialized right finger sensor')
        except Exception as e:
            self.node.get_logger().warning(f'Failed to initialize right finger sensor: {e}')
            self.finger_sensors['right'] = None

        self.position_sub = self.node.create_subscription(
            Float64,
            'gripper_position',
            self.gripper_position_callback,
            10
        )

        self.gripper_state_pub = self.node.create_publisher(
            Float64,
            'gripper_current_position',
            10
        )

        # Таймер для публикации состояния
        self.node.create_timer(0.1, self.publish_gripper_state)  # 10 Hz
        
        self.node.get_logger().info('Gripper initialized successfully')

    def gripper_position_callback(self, msg):
        position = msg.data
        self.node.get_logger().info(f'Setting gripper position: {position}')
        try:
            for motor in self.finger_motors.values():
                motor.setPosition(position)
            
            self.node.get_logger().info(f'Set gripper position: {position}')
            
        except Exception as e:
            self.node.get_logger().error(f'Error in set_position: {e}')

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
            self.node.get_logger().error(f'Error publishing gripper state: {e}')
            