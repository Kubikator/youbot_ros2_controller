import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class Lidar:
    def __init__(self, node, robot, timestep):
        self.node = node
        self.robot = robot
        self.timestep = timestep
        
        self.node.get_logger().info('Initializing Lidar...')
        
        try:
            # Инициализация лидара
            self.lidar = self.robot.getDevice('lidar_sensor')
            if self.lidar is None:
                self.node.get_logger().error('Lidar device not found')
                raise Exception('Lidar device not found')
                
            # Включение лидара
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
            
            # Получение параметров лидара
            self.lidar_width = self.lidar.getHorizontalResolution()
            self.fov = self.lidar.getFov()
            self.min_range = self.lidar.getMinRange()
            self.max_range = self.lidar.getMaxRange()
            
            self.node.get_logger().info(f'Lidar initialized: width={self.lidar_width}, fov={self.fov:.2f}')
            self.node.get_logger().info(f'Lidar range: {self.min_range:.2f} - {self.max_range:.2f} m')
            
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize lidar: {e}')
            raise
        
        # Публикатор для данных лидара
        self.lidar_pub = self.node.create_publisher(
            LaserScan,
            'scan',
            10
        )
        
        # Таймер для обновления данных лидара
        self.node.create_timer(0.1, self.publish_lidar_data)  # 10 Hz

        # Создаем публикатор для публикации смещения камеры
        self.tf_broadcaster = TransformBroadcaster(self.node)
        
        # Смещение лидара относительно базы робота (0.3 по X)
        self.lidar_offset_x = 0.3
        self.lidar_offset_y = 0.0
        self.lidar_offset_z = 0.0
        
        self.node.create_timer(0.1, self.broadcast_tf)  # 10 Hz
        
        self.node.get_logger().info('Lidar initialized successfully')

    def publish_lidar_data(self):
        try:
            # Получение данных с лидара
            ranges = self.lidar.getRangeImage()
            
            if ranges:
                # Создание LaserScan сообщения
                scan_msg = LaserScan()
                scan_msg.header.stamp = self.node.get_clock().now().to_msg()
                scan_msg.header.frame_id = 'lidar_link'  # Фрейм лидара
                
                # Параметры лидара - исправляем для правильной ориентации
                scan_msg.angle_min = -self.fov / 2.0
                scan_msg.angle_max = self.fov / 2.0
                scan_msg.angle_increment = self.fov / (self.lidar_width - 1)
                scan_msg.time_increment = 0.0  # Не используется в Webots
                scan_msg.scan_time = 0.1  # Время сканирования (соответствует таймеру)
                scan_msg.range_min = self.min_range
                scan_msg.range_max = self.max_range
                
                # Заполнение данных диапазонов
                # Инвертируем порядок данных для исправления зеркального отображения
                scan_msg.ranges = []
                reversed_ranges = list(reversed(ranges))
                
                for range_value in reversed_ranges:
                    if math.isinf(range_value):
                        scan_msg.ranges.append(float('inf'))
                    else:
                        scan_msg.ranges.append(float(range_value))
                
                # Публикация данных
                self.lidar_pub.publish(scan_msg)
                
        except Exception as e:
            self.node.get_logger().error(f'Error publishing lidar data: {e}')

    def broadcast_tf(self):
        try:
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'  # Фрейм базы робота
            t.child_frame_id = 'lidar_link'  # Фрейм лидара
            
            # Позиция лидара относительно базы
            t.transform.translation.x = self.lidar_offset_x
            t.transform.translation.y = self.lidar_offset_y
            t.transform.translation.z = self.lidar_offset_z
            
            # Ориентация (лидар смотрит вперед)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.node.get_logger().error(f'Error broadcasting TF: {e}')

    def shutdown(self):
        try:
            if self.lidar is not None:
                self.lidar.disable()
            self.node.get_logger().info('Lidar shutdown')
        except Exception as e:
            self.node.get_logger().error(f'Error shutting down lidar: {e}')