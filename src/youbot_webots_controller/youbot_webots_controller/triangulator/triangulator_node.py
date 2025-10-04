#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point, TransformStamped
from tf2_ros import TransformListener, Buffer
import numpy as np
import cv2
from typing import Tuple, Optional
from std_msgs.msg import Header
import math
from youbot_webots_controller.triangulator.triangulator import Triangulation

class StereoTriangulationNode(Node):
    def __init__(self):
        super().__init__('triangulator_node')
        
        width = 480
        height = 480
        fov = 1

        fx = (width/2) / (math.tan(fov/2))
        fy = fx

        # Параметры
        self.declare_parameter('camera_matrix', [fx, 0.0, width/2, 0.0, fy, height/2, 0.0, 0.0, 1.0])
        self.declare_parameter('left_camera_frame', 'left_camera_link')
        self.declare_parameter('right_camera_frame', 'right_camera_link')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('timeout_sec', 0.1)
        
        # Получаем параметры
        cam_matrix_params = self.get_parameter('camera_matrix').value
        self.left_camera_frame = self.get_parameter('left_camera_frame').value
        self.right_camera_frame = self.get_parameter('right_camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        queue_size = self.get_parameter('queue_size').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        
        # Инициализация матрицы камеры
        self.camera_matrix = np.array(cam_matrix_params).reshape(3, 3)
        self.get_logger().info(f"Camera matrix: {self.camera_matrix}")
        
        # Инициализация триангулятора
        self.triangulator = Triangulation(self.camera_matrix)
        
        # TF buffer и listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Хранилище данных
        self.left_point = None
        self.right_point = None
        self.last_left_stamp = None
        self.last_right_stamp = None
        
        # Подписки
        self.left_sub = self.create_subscription(
            Point,
            '/left/object_center',
            self.left_callback,
            queue_size
        )
        
        self.right_sub = self.create_subscription(
            Point,
            '/right/object_center',
            self.right_callback,
            queue_size
        )
        
        # Публикатор для триангулированной точки
        self.point_pub = self.create_publisher(
            PointStamped,
            '/triangulated_object_center',
            queue_size
        )
        
        # Таймер для обработки данных
        self.process_timer = self.create_timer(0.1, self.process_points)  # 10 Hz
        
        self.get_logger().info("Stereo triangulation node initialized")
    
    def left_callback(self, msg: Point):
        """Callback для левой камеры"""
        self.left_point = (msg.x, msg.y)
        self.last_left_stamp = self.get_clock().now()
        self.get_logger().debug(f"Left point received: ({self.left_point[0]:.2f}, {self.left_point[1]:.2f})")
    
    def right_callback(self, msg: Point):
        """Callback для правой камеры"""
        self.right_point = (msg.x, msg.y)
        self.last_right_stamp = self.get_clock().now()
        self.get_logger().debug(f"Right point received: ({self.right_point[0]:.2f}, {self.right_point[1]:.2f})")
    
    def get_camera_pose(self, camera_frame: str, target_frame: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Получаем позу камеры относительно целевого фрейма
        
        Returns:
            Tuple[R, t] или None если трансформация недоступна
        """
        try:
            # Получаем трансформацию (используем самое актуальное время)
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.timeout_sec)
            )
            
            # Извлекаем трансляцию
            t = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Извлекаем ротацию (кватернион в матрицу поворота)
            q = transform.transform.rotation
            R = self.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
            
            return R, t
            
        except Exception as e:
            self.get_logger().warn(f"Transform exception for {camera_frame}: {e}")
            return None
    
    def quaternion_to_rotation_matrix(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Конвертируем кватернион в матрицу поворота 3x3"""
        # Нормализуем кватернион
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0:
            return np.eye(3)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Вычисляем матрицу поворота
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])
        
        return R
    
    def apply_coordinate_correction(self, R: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        применяем коррекцию системы координат для OpenCV
        Сначала поворот вокруг исходной оси Y на +90°, 
        затем поворот вокруг НОВОЙ оси Z на -90°
        """
        # Создаем матрицы поворотов
        # Поворот вокруг исходной оси Y на +90°
        Ry_90 = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
        
        # Поворот вокруг оси Z на -90°
        # В новой системе после первого поворота
        Rz_minus_90_new = np.array([
            [0, 1, 0],
            [-1, 0, 0],
            [0, 0, 1]
        ])
        
        # Правильная композиция: сначала Ry, потом Rz в новой системе
        # Это эквивалентно R_total = Ry_90 @ Rz_minus_90_new
        correction_matrix = Ry_90 @ Rz_minus_90_new
        
        # Применяем коррекцию
        R_corrected = (R @ correction_matrix).T
        t_corrected = -R_corrected @ t
        
        return R_corrected, t_corrected
    
    def are_points_fresh(self) -> bool:
        """Проверяем, что точки достаточно свежие"""
        if self.last_left_stamp is None or self.last_right_stamp is None:
            return False
            
        current_time = self.get_clock().now()
        left_age = (current_time - self.last_left_stamp).nanoseconds * 1e-9
        right_age = (current_time - self.last_right_stamp).nanoseconds * 1e-9
        
        max_age = 1  # максимальный возраст точек в секундах
        
        return left_age < max_age and right_age < max_age
    
    def process_points(self):
        """Основной метод обработки - вызывается по таймеру"""
        # Проверяем, есть ли данные с обеих камер и они свежие
        if (self.left_point is None or self.right_point is None or 
            not self.are_points_fresh()):
            return
        
        try:
            # Получаем позы камер
            left_pose = self.get_camera_pose(self.left_camera_frame, self.world_frame)
            right_pose = self.get_camera_pose(self.right_camera_frame, self.world_frame)
            
            if left_pose is None or right_pose is None:
                self.get_logger().warn("Could not get camera poses, skipping triangulation")
                return
            
            # Применяем коррекцию системы координат
            R_left, t_left = self.apply_coordinate_correction(*left_pose)
            R_right, t_right = self.apply_coordinate_correction(*right_pose)

            # self.get_logger().info(f"Left camera pose after correction is: {R_left} {t_left}")
            # self.get_logger().info(f"Right camera pose after correction is: {R_right} {t_right}")
            
            # Подготавливаем точки для триангуляции
            left_point_np = np.array([self.left_point[0], self.left_point[1]])
            right_point_np = np.array([self.right_point[0], self.right_point[1]])
            
            # Выполняем триангуляцию
            object_3d = self.triangulator.triangulate_single_object(
                (R_left, t_left),
                (R_right, t_right),
                left_point_np,
                right_point_np
            )
            
            # Публикуем результат
            self.publish_triangulated_point(object_3d)
            
            # self.get_logger().info(
            #     f"Triangulated object at: ({object_3d[0]:.3f}, {object_3d[1]:.3f}, {object_3d[2]:.3f})"
            # )
            
        except Exception as e:
            self.get_logger().error(f"Error in triangulation: {e}")
    
    def publish_triangulated_point(self, point_3d: np.ndarray):
        """Публикует триангулированную точку в топик"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.point.x = float(point_3d[0])
        msg.point.y = float(point_3d[1])
        msg.point.z = float(point_3d[2])
        
        self.point_pub.publish(msg)
        
        # self.get_logger().debug(
        #     f"Published triangulated point: "
        #     f"x={point_3d[0]:.3f}, y={point_3d[1]:.3f}, z={point_3d[2]:.3f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    
    node = StereoTriangulationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()