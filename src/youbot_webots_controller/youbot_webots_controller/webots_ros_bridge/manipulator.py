import rclpy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class Manipulator:
    def __init__(self, node, robot, timestep):
        self.node = node
        self.robot = robot
        self.timestep = timestep
        
        self.node.get_logger().info('Initializing Manipulator...')
        
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
                self.node.get_logger().info(f'Initialized arm motor: {joint_name}')
            except Exception as e:
                self.node.get_logger().error(f'Failed to initialize arm motor {joint_name}: {e}')
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
                    self.node.get_logger().info(f'Initialized position sensor: {sensor_name}')
                else:
                    self.node.get_logger().warning(f'Position sensor {sensor_name} not found')
                    self.position_sensors.append(None)
            except Exception as e:
                self.node.get_logger().error(f'Failed to initialize sensor {joint_name}: {e}')
                self.position_sensors.append(None)
        
        
        # Подписчик для получения целевых позиций манипулятора
        self.arm_target_sub = self.node.create_subscription(
            Float64MultiArray,
            'arm_target_positions',
            self.arm_target_callback,
            10
        )
        
        # Публикатор для состояний суставов
        self.joint_state_pub = self.node.create_publisher(
            JointState,
            'arm_joints_states',
            10
        )
        
        # Таймер для обновления состояний суставов
        self.node.create_timer(0.1, self.update_joint_states)  # 10 Hz
        
        self.node.get_logger().info('Manipulator initialized successfully')

    def arm_target_callback(self, msg):
        try:
            if len(msg.data) == len(self.arm_motors):
                for i, motor in enumerate(self.arm_motors):
                    motor.setPosition(msg.data[i])
            else:
                self.node.get_logger().warn(f'Received {len(msg.data)} positions, expected {len(self.arm_motors)}')
                
        except Exception as e:
            self.node.get_logger().error(f'Error in arm_target_callback: {e}')

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
            joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
            joint_state_msg.name = self.arm_joint_names
            joint_state_msg.position = current_positions
            
            self.joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.node.get_logger().error(f'Error updating joint states: {e}')
