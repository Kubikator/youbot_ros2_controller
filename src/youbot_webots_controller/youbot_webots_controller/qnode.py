import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, QByteArray
from cv_bridge import CvBridge
import cv2
import os

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")


class QNode(QObject):
    # Сигнал для передачи изображения в GUI
    image_received = pyqtSignal(object)  # Будем передавать QImage или QPixmap
    
    def __init__(self, node_name='QNode'):
        super().__init__()
        
        if not rclpy.ok():
            rclpy.init()
            
        self.node = Node(node_name)
        self.bridge = CvBridge()
        
        self.cmd_vel_publisher = self.node.create_publisher(
            Twist, 'cmd_vel', 10
        )
        
        # Подписка на топик с изображениями
        self.image_subscription = self.node.create_subscription(
            Image,
            'left/image_processed',
            self.image_callback,
            10
        )
        
        # Таймер для обработки ROS2 callbacks
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_once)
        self.ros_timer.start(10)
        
        self.get_logger().info(f"'{node_name}' successfully initialized")
        self.get_logger().info("Subscribed to 'processed_image' topic")
        
    def get_logger(self):
        return self.node.get_logger()
        
    def spin_once(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.001)
        except Exception as e:
            self.get_logger().error(f"Error in spin_once: {e}")
    
    def image_callback(self, msg):
        """Обработчик сообщений с изображениями"""
        try:
            # Конвертируем ROS Image в OpenCV изображение
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Конвертируем OpenCV изображение в QImage
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            
            # Конвертируем BGR в RGB для QImage
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Создаем QImage
            from PyQt5.QtGui import QImage
            qimage = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # Делаем копию, так как данные могут быть освобождены
            qimage_copy = qimage.copy()
            
            # Отправляем изображение через сигнал
            self.image_received.emit(qimage_copy)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            
    def publish_velocity(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                        angular_x=0.0, angular_y=0.0, angular_z=0.0):
        try:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.linear.y = float(linear_y)
            msg.linear.z = float(linear_z)
            msg.angular.x = float(angular_x)
            msg.angular.y = float(angular_y)
            msg.angular.z = float(angular_z)
            
            self.cmd_vel_publisher.publish(msg)
            
            if any([linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]):
                self.get_logger().debug(
                    f"Published: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), "
                    f"angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})"
                )
        except Exception as e:
            self.get_logger().error(f"Publishing error: {e}")
        
    def stop_robot(self):
        self.publish_velocity()
        self.get_logger().info("Robot stopped")
        
    def shutdown(self):
        try:
            self.stop_robot()
            self.ros_timer.stop()
            
            if self.node:
                self.node.destroy_node()
                self.get_logger().info("QNode finished job")
                
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Failed to shutdown QNode: {e}")