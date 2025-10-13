#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        
        self.bridge = CvBridge()
        self.save_counter = 0
        self.save_dir = "yolo_training_images"
        
        # Создаем папку если нет
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # Подписываемся на топик с изображением
        self.subscription = self.create_subscription(
            Image,
            '/left/image_raw',  # Измени если твой топик называется иначе
            self.image_callback,
            10
        )
        
        # Таймер для сохранения каждую секунду
        self.timer = self.create_timer(1.0, self.save_image)
        self.current_frame = None
        
        self.get_logger().info(f"Image saver started! Saving to: {self.save_dir}")
        self.get_logger().info("Saving 1 image per second automatically...")

    def image_callback(self, msg):
        """Получаем кадры из топика"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def save_image(self):
        """Сохраняем текущий кадр"""
        if self.current_frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"img_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            
            success = cv2.imwrite(filepath, self.current_frame)
            
            if success:
                self.save_counter += 1
                if self.save_counter % 50 == 0:  # Логируем каждые 50 кадров
                    self.get_logger().info(f"Saved {self.save_counter} images...")
            else:
                self.get_logger().error(f"Failed to save: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Finished! Total images saved: {node.save_counter}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()