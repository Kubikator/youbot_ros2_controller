#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Thread, Lock
import time
from geometry_msgs.msg import Point
from youbot_webots_controller.object_detector.async_object_detector import AsyncObjectDetector

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        # Инициализация CV Bridge
        self.bridge = CvBridge()

        # Создаем классы асинхронной обработки изображений для двух камер
        self.get_logger().info('Start to initialize object detectors...')
        self.left_processor = AsyncObjectDetector('src/youbot_webots_controller/models/yolo11n.pt',confidence_threshold=0.6)
        self.right_processor = AsyncObjectDetector('src/youbot_webots_controller/models/yolo11n.pt',confidence_threshold=0.6)
        self.get_logger().info('Finish to initialize object detectors')
        
        # Подписка на топики с изображениями левой и правой камер
        self.left_subscription = self.create_subscription(
            Image,
            'left/image_raw',
            self.left_image_callback,
            10
        )
        
        self.right_subscription = self.create_subscription(
            Image,
            'right/image_raw',
            self.right_image_callback,
            10
        )

        # Публикаторы топиков с обработанными изображениями
        self.left_image_pub = self.create_publisher(Image, 'left/image_processed', 10)
        self.right_image_pub = self.create_publisher(Image, 'right/image_processed', 10)

        # Публикаторы топиков с центрами объектов
        self.left_center_pub = self.create_publisher(Point, 'left/object_center', 10)
        self.right_center_pub = self.create_publisher(Point, 'right/object_center', 10)

        self.left_orange_center_pub = self.create_publisher(Point, 'left/orange_center', 10)
        self.right_orange_center_pub = self.create_publisher(Point, 'right/orange_center', 10)

        self.left_apple_center_pub = self.create_publisher(Point, 'left/apple_center', 10)
        self.right_apple_center_pub = self.create_publisher(Point, 'right/apple_center', 10)

        self.left_cup_center_pub = self.create_publisher(Point, 'left/cup_center', 10)
        self.right_cup_center_pub = self.create_publisher(Point, 'right/cup_center', 10)

        self.left_wine_center_pub = self.create_publisher(Point, 'left/wine_center', 10)
        self.right_wine_center_pub = self.create_publisher(Point, 'right/wine_center', 10)

        # Разделяемые данные для левой камеры
        self._left_thread = Thread(target=self._left_run, daemon=True)
        self._left_lock = Lock()
        self._left_current_image = None
        self._left_flag_new = False

        # Разделяемые данные для правой камеры
        self._right_thread = Thread(target=self._right_run, daemon=True)
        self._right_lock = Lock()
        self._right_current_image = None
        self._right_flag_new = False

        # Запускаем потоки обработки
        self._left_thread.start()
        self._right_thread.start()
        
        self.get_logger().info('Image Processor node initialized')
        self.get_logger().info('Waiting for images on topics: /left/image_raw, /right/image_raw')

    def left_image_callback(self, msg):
        """Обработчик для левой камеры"""
        try:
            # Конвертируем ROS Image сообщение в OpenCV изображение
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Сохраняем изображение для дальнейшей обработки
            with self._left_lock:
                self._left_flag_new = True
                self._left_current_image = cv_image
            
            # Логируем информацию о полученном изображении
            #self.get_logger().info(f'Left camera - Received image: {cv_image.shape}, type: {cv_image.dtype}')

            
        except Exception as e:
            self.get_logger().error(f'Left camera - Error converting image: {e}')

    def right_image_callback(self, msg):
        """Обработчик для правой камеры"""
        try:
            # Конвертируем ROS Image сообщение в OpenCV изображение
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Сохраняем изображение для дальнейшей обработки
            with self._right_lock:
                self._right_flag_new = True
                self._right_current_image = cv_image
            
            # Логируем информацию о полученном изображении
            #self.get_logger().info(f'Right camera - Received image: {cv_image.shape}, type: {cv_image.dtype}')

            
        except Exception as e:
            self.get_logger().error(f'Right camera - Error converting image: {e}')

    def _publish_object_center(self, center_x, center_y, publisher, camera_side):
        """Публикует центр объекта в топик"""
        try:
            point_msg = Point()
            point_msg.x = float(center_x)
            point_msg.y = float(center_y)
            point_msg.z = 0.0  # z=0 для 2D точки
            
            publisher.publish(point_msg)
            
            # Логируем только иногда чтобы не засорять консоль
            if not hasattr(self, f'{camera_side.lower()}_center_log_counter'):
                setattr(self, f'{camera_side.lower()}_center_log_counter', 0)
            
            counter = getattr(self, f'{camera_side.lower()}_center_log_counter')
            counter += 1
            setattr(self, f'{camera_side.lower()}_center_log_counter', counter)
            
            if counter >= 30:  # Каждые 30 публикаций
                self.get_logger().info(f'{camera_side} camera - Published object center: ({center_x:.1f}, {center_y:.1f})')
                setattr(self, f'{camera_side.lower()}_center_log_counter', 0)
                
        except Exception as e:
            self.get_logger().error(f'{camera_side} camera - Error publishing object center: {e}')

    def _left_run(self):
        """Поток обработки для левой камеры"""
        while True:
            with self._left_lock:
                flag = self._left_flag_new

            if flag:
                if self.left_processor.is_completed():
                    # Получаем обработанное изображение и объекты
                    processed_image, objects = self.left_processor.get_result()

                    # Публикуем обработанное изображение
                    image_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = 'left_camera_link'
                    self.left_image_pub.publish(image_msg)

                    classes = self.left_processor.get_classes()
                    for cl in classes:
                        center = self.left_processor.get_object_centers_by_class(cl)
                        if len(center) != 0:
                            center_x, center_y = center[0][0], center[0][1]
                        else:
                            center_x, center_y = float('nan'), float('nan')

                        if cl == 'orange':
                            self._publish_object_center(center_x, center_y, self.left_orange_center_pub, "Left")
                        elif cl == 'apple':
                            self._publish_object_center(center_x, center_y, self.left_apple_center_pub, "Left")
                        elif cl == 'cup':
                            self._publish_object_center(center_x, center_y, self.left_cup_center_pub, "Left")
                        elif cl == 'wine glass':
                            self._publish_object_center(center_x, center_y, self.left_wine_center_pub, "Left")
                    
                    result = self.left_processor.get_highest_confidence_center()
                    if result is not None:
                        center_x, center_y = result[0], result[1]
                    else:
                        center_x, center_y = float('nan'), float('nan')
                    self._publish_object_center(center_x, center_y, self.left_center_pub, "Left")
                
                if self.left_processor.is_idle():
                    with self._left_lock:
                        self._left_flag_new = False
                        image = self._left_current_image

                    self.left_processor.process_image(image)
            
            time.sleep(0.01)

    def _right_run(self):
        """Поток обработки для правой камеры"""
        while True:
            with self._right_lock:
                flag = self._right_flag_new

            if flag:
                if self.right_processor.is_completed():
                    # Получаем обработанное изображение и объекты
                    processed_image, objects = self.right_processor.get_result()

                    # Публикуем обработанное изображение
                    image_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = 'right_camera_link'
                    self.right_image_pub.publish(image_msg)

                    # Получаем все классы и проходимся по каждому
                    classes = self.right_processor.get_classes()
                    for cl in classes:
                        center = self.right_processor.get_object_centers_by_class(cl)
                        if len(center) != 0:
                            center_x, center_y = center[0][0], center[0][1]
                        else:
                            center_x, center_y = float('nan'), float('nan')

                        if cl == 'orange':
                            self._publish_object_center(center_x, center_y, self.right_orange_center_pub, "Right")
                        elif cl == 'apple':
                            self._publish_object_center(center_x, center_y, self.right_apple_center_pub, "Right")
                        elif cl == 'cup':
                            self._publish_object_center(center_x, center_y, self.right_cup_center_pub, "Right")
                        elif cl == 'wine glass':
                            self._publish_object_center(center_x, center_y, self.right_wine_center_pub, "Right")
                    
                    result = self.right_processor.get_highest_confidence_center()
                    if result is not None:
                        center_x, center_y = result[0], result[1]
                    else:
                        center_x, center_y = float('nan'), float('nan')
                    self._publish_object_center(center_x, center_y, self.right_center_pub, "Right")
                
                if self.right_processor.is_idle():
                    with self._right_lock:
                        self._right_flag_new = False
                        image = self._right_current_image

                    self.right_processor.process_image(image)
            
            time.sleep(0.01)

    def shutdown(self):
        """Завершение работы"""
        cv2.destroyAllWindows()
        self.get_logger().info('Image Processor node shutdown')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        image_processor = ImageProcessor()
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'image_processor' in locals():
            image_processor.shutdown()
        image_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()