import rclpy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math

class MobilePlatform:
    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        
        self.node.get_logger().info('Initializing Mobile Platform with GPS...')
        
        # Инициализация моторов колес
        self.wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
        self.wheels = []
        
        # Инициализация GPS для абсолютных координат
        self.gps = None
        try:
            self.gps = self.robot.getDevice('gps')
            if self.gps:
                self.gps.enable(int(self.robot.getBasicTimeStep()))
                self.node.get_logger().info('GPS initialized successfully')
            else:
                self.node.get_logger().error('GPS device not found!')
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize GPS: {e}')
        
        # Инициализация IMU для ориентации
        self.imu = None
        try:
            self.imu = self.robot.getDevice('inertial unit')
            if self.imu:
                self.imu.enable(int(self.robot.getBasicTimeStep()))
                self.node.get_logger().info('IMU initialized successfully')
            else:
                self.node.get_logger().warning('IMU device not found, using GPS only')
        except Exception as e:
            self.node.get_logger().warning(f'Failed to initialize IMU: {e}')
        
        for name in self.wheel_names:
            try:
                # Инициализация мотора
                wheel = self.robot.getDevice(name)
                wheel.setPosition(float('inf'))  # Режим управления скоростью
                wheel.setVelocity(0.0)
                self.wheels.append(wheel)
                self.node.get_logger().info(f'Initialized wheel motor: {name}')
                    
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
        
        # Публикатор для абсолютных координат
        self.gps_pose_pub = self.node.create_publisher(
            PoseStamped,
            'gps_pose',
            10
        )
        
        # Таймер для обновления и публикации абсолютных координат
        self.node.create_timer(0.05, self.update_gps_pose)  # 20 Hz
        
        self.node.get_logger().info('Mobile Platform with GPS initialized successfully')

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

    def update_gps_pose(self):
        """
        Публикация абсолютных координат из GPS и IMU
        """
        try:
            if not self.gps:
                return
                
            current_time = self.node.get_clock().now()
            
            # Получение данных с GPS
            gps_values = self.gps.getValues()
            
            # Создание сообщения с абсолютными координатами
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time.to_msg()
            pose_msg.header.frame_id = 'map'
            
            # Позиция из GPS (Webots GPS возвращает [x, y, z])
            pose_msg.pose.position = Point(
                x=float(gps_values[0]),  # Восток (East)
                y=float(gps_values[1]),  # Север (North) - в Webots Z это высота, Y это север
                z=0.0  # Мы работаем в 2D
            )
            
            # Ориентация из IMU (если доступен)
            if self.imu:
                imu_values = self.imu.getRollPitchYaw()
                # Обычно IMU возвращает [roll, pitch, yaw]
                yaw = imu_values[2]  # Yaw угол вокруг вертикальной оси
                
                # Преобразование угла Эйлера в кватернион
                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                cp = math.cos(0.0)  # pitch = 0 для 2D
                sp = math.sin(0.0)
                cr = math.cos(0.0)  # roll = 0 для 2D
                sr = math.sin(0.0)
                
                pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
                pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
                pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
                pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
            else:
                # Если IMU нет, используем ориентацию по умолчанию
                pose_msg.pose.orientation.w = 1.0
            
            # Публикация абсолютных координат
            self.gps_pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.node.get_logger().error(f'Error updating GPS pose: {e}')

    def shutdown(self):
        # Останавливаем колеса при выходе
        for wheel in self.wheels:
            if wheel is not None:
                wheel.setVelocity(0.0)
        self.node.get_logger().info('Mobile Platform shutdown')