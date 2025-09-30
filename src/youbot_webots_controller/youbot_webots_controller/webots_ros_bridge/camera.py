import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import numpy as np
from controller import Camera
import time
import os
from datetime import datetime
import cv2

# Используем только стандартный сервис Trigger
from std_srvs.srv import Trigger


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
        self.camera_offset_z = 0.1  # Высота по Z

        self.current_image = None
        self.save_counter = 0
        self.default_save_dir = "camera_images"  # Папка для сохранения по умолчанию
        
        # Создаем папку для сохранения изображений
        self._create_save_directory()
        
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
        
        # Сервис для сохранения изображений
        self.save_service = self.node.create_service(
            Trigger, 
            'save_image', 
            self.handle_save_image
        )
        
        # Таймеры
        self.node.create_timer(1, self.publish_image)
        self.node.create_timer(0.1, self.publish_tf)     # 10 Hz для TF
        
        self.node.get_logger().info('Webots Camera initialized successfully')
        self.node.get_logger().info('Save image service available: /save_image')

    def _create_save_directory(self):
        """Создает папку для сохранения изображений если она не существует"""
        if not os.path.exists(self.default_save_dir):
            os.makedirs(self.default_save_dir)
            self.node.get_logger().info(f'Created directory: {self.default_save_dir}')

    def handle_save_image(self, request, response):
        """
        Обработчик сервиса Trigger для сохранения текущего изображения
        """
        try:
            if self.current_image is None:
                response.success = False
                response.message = "No image available to save"
                self.node.get_logger().warn("Attempted to save image but no image available")
                return response
            
            # Генерируем имя файла с временной меткой
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Добавляем миллисекунды
            filename = f"webots_camera_{timestamp}.png"
            filepath = os.path.join(self.default_save_dir, filename)
            
            # Сохраняем изображение в формате PNG
            success = cv2.imwrite(filepath, self.current_image)
            
            if success:
                self.save_counter += 1
                response.success = True
                response.message = f"Image saved successfully: {filepath}"
                self.node.get_logger().info(f"Saved image: {filepath}")
            else:
                response.success = False
                response.message = f"Failed to save image: {filepath}"
                self.node.get_logger().error(f"Failed to save image: {filepath}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error saving image: {str(e)}"
            self.node.get_logger().error(f"Error in save_image service: {e}")
            
        return response

    def save_current_image(self, filename=None):
        """
        Публичный метод для сохранения текущего изображения
        Можно вызывать из других частей кода
        """
        try:
            if self.current_image is None:
                self.node.get_logger().warn("No image available to save")
                return False, "No image available"
            
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"webots_camera_{timestamp}.png"
                filepath = os.path.join(self.default_save_dir, filename)
            else:
                # Добавляем расширение .png если не указано
                if not filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                    filename += '.png'
                filepath = filename
            
            # Создаем директорию если нужно
            os.makedirs(os.path.dirname(filepath) if os.path.dirname(filepath) else '.', exist_ok=True)
            
            success = cv2.imwrite(filepath, self.current_image)
            
            if success:
                self.save_counter += 1
                self.node.get_logger().info(f"Saved image: {filepath}")
                return True, filepath
            else:
                self.node.get_logger().error(f"Failed to save image: {filepath}")
                return False, f"Failed to save image: {filepath}"
                
        except Exception as e:
            error_msg = f"Error saving image: {str(e)}"
            self.node.get_logger().error(error_msg)
            return False, error_msg

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
            self.current_image = cv_image
            
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