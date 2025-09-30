#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from youbot_webots_controller.object_detector.async_object_detector import AsyncObjectDetector

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        # Инициализация CV Bridge
        self.bridge = CvBridge()

        # Создаем класс асинхронной обработки изображений
        self.get_logger().info('Start to initialize object detector...')
        self.processor = AsyncObjectDetector('src/youbot_webots_controller/models/yolo11n.pt')
        self.get_logger().info('Finish to initialize object detector')
        
        # Подписка на топик с изображением
        self.subscription = self.create_subscription(
            Image,
            'image_raw',  # Тот же топик, что публикует WebotsCamera
            self.image_callback,
            10  # QoS profile depth
        )

        # Публикатор топиков с обработанным изображением
        self.image_pub = self.create_publisher(Image, 'image_processed', 10)

        # Переменная для хранения текущего изображения
        self.current_image = None
        self.processed_image = None
        self.objects = None

        
        self.get_logger().info('Image Processor node initialized')
        self.get_logger().info('Waiting for images on topic: /image_raw')

    def image_callback(self, msg):

        try:
            # Конвертируем ROS Image сообщение в OpenCV изображение
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Сохраняем изображение для дальнейшей обработки
            self.current_image = cv_image
            
            # Логируем информацию о полученном изображении
            self.get_logger().info(f'Received image: {cv_image.shape}, type: {cv_image.dtype}')

            self.process_image()
            
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def process_image(self):
        if self.processor.is_completed():
            self.processed_image, self.objects = self.processor.get_result()
            cv2.imwrite("result_my_before.png", self.processed_image)

            image_msg = self.bridge.cv2_to_imgmsg(self.processed_image, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_link'  # Используем тот же фрейм, что и в TF
            self.image_pub.publish(image_msg)
            
        if self.processor.is_idle():
            self.processor.process_image(self.current_image)

    def shutdown(self):

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