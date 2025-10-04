import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class GPSOdometryNode(Node):
    def __init__(self):
        super().__init__('youbot_odometry')
        
        self.get_logger().info('Initializing GPS Odometry Node...')
        
        # Текущая и предыдущая позиции
        self.current_pose = None
        self.previous_pose = None
        self.first_message = True
        
        # Система координат одометрии
        self.odom_pose = None
        self.initial_pose = None
        
        # Подписчик на абсолютные координаты
        self.gps_sub = self.create_subscription(
            PoseStamped,
            'gps_pose',
            self.gps_pose_callback,
            10
        )
        
        # Публикаторы
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info('GPS Odometry Node initialized successfully')

    def gps_pose_callback(self, msg):
        """
        Обработка абсолютных координат и преобразование в одометрию
        """
        try:
            current_time = self.get_clock().now()
            self.current_pose = msg.pose
            
            if self.first_message:
                # Первое сообщение - инициализируем систему одометрии
                self.initial_pose = self.current_pose
                self.odom_pose = self.current_pose
                self.previous_pose = self.current_pose
                self.first_message = False
                return
            
            # Вычисляем смещение относительно начальной позиции
            # Это и есть наша одометрия в системе 'odom'
            dx = self.current_pose.position.x - self.initial_pose.position.x
            dy = self.current_pose.position.y - self.initial_pose.position.y
            
            # Для простоты используем текущую ориентацию (можно вычислять и скорость)
            orientation = self.current_pose.orientation
            
            # Публикуем одометрию
            self.publish_odometry(dx, dy, orientation, current_time)
            
            # Обновляем предыдущую позицию
            self.previous_pose = self.current_pose
            
        except Exception as e:
            self.get_logger().error(f'Error in GPS pose callback: {e}')

    def publish_odometry(self, x, y, orientation, current_time):
        """
        Публикация одометрии и TF трансформа
        """
        try:
            # Создание сообщения Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # Позиция в системе одометрии
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = orientation
            
            # Ковариация (очень маленькая, т.к. GPS точный в симуляции)
            odom_msg.pose.covariance = [0.001] * 36
            odom_msg.pose.covariance[0] = 0.0001  # x
            odom_msg.pose.covariance[7] = 0.0001  # y
            odom_msg.pose.covariance[35] = 0.001  # yaw
            
            # Скорость (можно вычислять из разности позиций, если нужно)
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0
            
            # Публикация одометрии
            self.odom_pub.publish(odom_msg)
            
            # Публикация TF трансформа odom -> base_link
            transform = TransformStamped()
            transform.header.stamp = current_time.to_msg()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = orientation
            
            self.tf_broadcaster.sendTransform(transform)
            
            # Логирование для отладки
            self.get_logger().debug(
                f'Odom: x={x:.3f}, y={y:.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')

    def reset_odometry(self):
        """
        Сброс одометрии (установка текущей позиции как начала координат)
        """
        if self.current_pose:
            self.initial_pose = self.current_pose
            self.get_logger().info('Odometry reset')

def main(args=None):
    rclpy.init(args=args)
    node = GPSOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()