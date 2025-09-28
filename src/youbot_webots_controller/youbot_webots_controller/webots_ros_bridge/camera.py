import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import numpy as np
from controller import Camera
import time

class WebotsCamera:
    def __init__(self, node, camera_name='camera', sampling_period=16):
        self.node = node
        self.camera_name = camera_name
        
        self.node.get_logger().info(f'Initializing Webots Camera: {camera_name}...')
        
        # Инициализация CV Bridge
        self.bridge = CvBridge()
        
        # Инициализация TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self.node)
        
        # Параметры трансформации камеры
        self.camera_offset_x = 0.3  # Смещение вперед по X
        self.camera_offset_y = 0.0  # По центру по Y
        self.camera_offset_z = 0.2  # Высота по Z
        
        try:
            # Инициализация камеры через Webots API
            self.camera = Camera(name=camera_name, sampling_period=sampling_period)
            self.camera.enable(sampling_period)
            
            # Получаем параметры камеры
            self.width = self.camera.getWidth()
            self.height = self.camera.getHeight()
            
            self.node.get_logger().info(f'Camera: {self.width}x{self.height}')
            self.node.get_logger().info(f'Camera TF: base_link -> camera_link')
            self.node.get_logger().info(f'Camera offset: x={self.camera_offset_x}, y={self.camera_offset_y}, z={self.camera_offset_z}')
            
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize camera: {e}')
            raise
        
        self.camera_first_time = True
        # Публикаторы
        self.image_pub = self.node.create_publisher(Image, 'image_raw', 10)
        
        # Таймеры
        self.node.create_timer(1, self.publish_image)
        self.node.create_timer(0.1, self.publish_tf)     # 10 Hz для TF
        
        self.node.get_logger().info('Webots Camera initialized successfully')

    def publish_tf(self):
        try:
            t = TransformStamped()
            
            # Заполняем заголовок
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'    # Родительский фрейм
            t.child_frame_id = 'camera_link'   # Дочерний фрейм (камера)
            
            # Позиция камеры относительно base_link
            t.transform.translation.x = self.camera_offset_x
            t.transform.translation.y = self.camera_offset_y
            t.transform.translation.z = self.camera_offset_z
            
            # Ориентация камеры
            # Камера смотрит вперед по оси X
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            # Публикуем трансформацию
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.node.get_logger().error(f'Error publishing TF: {e}')

    def detect_format(self, image_data):
        array = np.frombuffer(image_data, np.uint8)
        total_pixels = self.width * self.height
        
        self.node.get_logger().info(f'Total pixels: {total_pixels}')
        self.node.get_logger().info(f'Data length: {len(array)}')
        self.node.get_logger().info(f'Bytes per pixel: {len(array) / total_pixels:.2f}')
        
        if len(array) == total_pixels * 4:
            self.node.get_logger().info('Detected format: BGRA (4 bytes per pixel)')
            self.image_format = 'bgra8'
        elif len(array) == total_pixels * 3:
            self.node.get_logger().info('Detected format: BGR (3 bytes per pixel)')
            self.image_format = 'bgr8'
        elif len(array) == total_pixels:
            self.node.get_logger().info('Detected format: Grayscale (1 byte per pixel)')
            self.image_format = 'mono8'
        else:
            self.node.get_logger().warning(f'Unknown format: {len(array)} bytes for {total_pixels} pixels')
            self.image_format = 'mono8'

    def convert_image(self, image_data):
        try:
            array = np.frombuffer(image_data, np.uint8)
            total_pixels = self.width * self.height
            
            if len(array) == total_pixels * 4:
                # BGRA format
                image_bgra = array.reshape((self.height, self.width, 4))
                image_bgr = image_bgra[:, :, :3]  # Убираем альфа-канал
                return image_bgr, 'bgr8'
                
            elif len(array) == total_pixels * 3:
                # BGR format
                image_bgr = array.reshape((self.height, self.width, 3))
                return image_bgr, 'bgr8'
                
            elif len(array) == total_pixels:
                # Grayscale
                image_gray = array.reshape((self.height, self.width))
                return image_gray, 'mono8'
                
            else:
                # Fallback - создаем черное изображение
                self.node.get_logger().warning(f'Unexpected format: {len(array)} bytes')
                black_image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                return black_image, 'bgr8'
                
        except Exception as e:
            self.node.get_logger().error(f'Error converting image: {e}')
            # Fallback изображение
            black_image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            return black_image, 'bgr8'

    def publish_image(self):
        try:
            if (self.camera_first_time):
                self.camera_first_time = False
                # Тестовый вызов для определения формата
                test_image = self.camera.getImage()
                if test_image:
                    self.node.get_logger().info(f'Image data length: {len(test_image)} bytes')
                    self.detect_format(test_image)

            # Получаем изображение из Webots
            image_data = self.camera.getImage()
            if image_data is None:
                self.node.get_logger().warning('Camera returned None image')
                return
            
            # Конвертируем байты в OpenCV изображение
            cv_image, encoding = self.convert_image(image_data)
            
            # Конвертируем в ROS2 сообщение
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
            image_msg.header.stamp = self.node.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_link'  # Используем тот же фрейм, что и в TF
            
            # Публикуем
            self.image_pub.publish(image_msg)
            
            # Логируем размер (только первые несколько раз)
            if not hasattr(self, 'log_counter'):
                self.log_counter = 0
            if self.log_counter < 5:
                self.node.get_logger().info(f'Published image: {encoding}, shape: {cv_image.shape}')
                self.log_counter += 1
                
        except Exception as e:
            self.node.get_logger().error(f'Error publishing image: {e}')

    def shutdown(self):
        try:
            if self.camera is not None:
                self.camera.disable()
            self.node.get_logger().info('Camera shutdown')
        except Exception as e:
            self.node.get_logger().error(f'Error shutting down camera: {e}')