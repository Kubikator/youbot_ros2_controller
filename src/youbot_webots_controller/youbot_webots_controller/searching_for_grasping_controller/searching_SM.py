#!/usr/bin/env python3

import rclpy
import sys
import smach
import smach_ros
from geometry_msgs.msg import Twist, Point, PointStamped, Pose
from std_msgs.msg import Float64
from youbot_webots_controller.srv import CoordinateTransform
import time
import math
import threading
import numpy as np

class InitialState(smach.State):
    def __init__(self, target_object):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target_object = target_object
        
    def execute(self, userdata):
        # Создаем временный узел
        node = rclpy.create_node('initial_state')
        
        try:
            node.get_logger().info('=== ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ ===')
            node.get_logger().info(f'Целевой объект: {self.target_object}')
            
            node.get_logger().info('Система готова к работе')
            return 'success'
            
        finally:
            # Всегда уничтожаем узел
            node.destroy_node()


class SearchingState(smach.State):
    def __init__(self, target_object):
        smach.State.__init__(self, outcomes=['object_found', 'failure'])
        self.target_object = target_object
        self.flag_left = False
        self.flag_right = False
        self.node = None
        self.published_left = False
        self.published_right = False
        
    def execute(self, userdata):
        # Создаем временный узел
        self.node = rclpy.create_node('searching_state')
        
        try:
            self.node.get_logger().info(f'=== ПОИСК ОБЪЕКТА: {self.target_object} ===')
            self.node.get_logger().info('Вращение вокруг оси для поиска...')
            
            # Создаем издателя для cmd_vel
            cmd_vel_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

            name_left = ''
            name_right = ''

            if self.target_object == 'blue_cyl':
                name_left = 'left/blue_cyl_center'
                name_right = 'right/blue_cyl_center'
            elif self.target_object == 'green_cyl':
                name_left = 'left/green_cyl_center'
                name_right = 'right/green_cyl_center'

            # Подписка на топики с изображениями левой и правой камер
            self.left_subscription = self.node.create_subscription(
                Point,
                name_left,
                self.left_callback,
                10
            )

            self.right_subscription = self.node.create_subscription(
                Point,
                name_right,
                self.right_callback,
                10
            )
            
            # Даем время для инициализации издателя
            import time
            time.sleep(0.5)
            
            # Публикуем команду вращения
            twist_msg = Twist()
            twist_msg.angular.z = 0.3  # rad/s
            cmd_vel_publisher.publish(twist_msg)
            cmd_vel_publisher.publish(twist_msg)
            cmd_vel_publisher.publish(twist_msg)
            self.published_right = True
            
            # Здесь будет реальная логика поиска через ваш Action сервер
            self.node.get_logger().info(f'Ищем: {self.target_object}')
            
            while (not self.flag_left) or (not self.flag_right):
                if self.flag_left and not self.published_left:
                    self.node.get_logger().info('Вращаюсь по часовой')
                    twist_msg = Twist()
                    twist_msg.angular.z = 0.3  # rad/s
                    cmd_vel_publisher.publish(twist_msg)
                    self.published_left = True
                    self.published_right = False
                elif self.flag_right and not self.published_right:
                    self.node.get_logger().info('Вращаюсь против часовой')
                    twist_msg = Twist()
                    twist_msg.angular.z = -0.3  # rad/s
                    cmd_vel_publisher.publish(twist_msg)
                    self.published_left = False
                    self.published_right = True
                self.delay_with_callbacks(node=self.node, delay_seconds=0.1)
            
            self.node.get_logger().info('Объект обнаружен в обоих камерах')
            stop_msg = Twist()
            cmd_vel_publisher.publish(stop_msg)
            return 'object_found'
            
        except Exception as e:
            self.node.get_logger().error(f'Ошибка в состоянии поиска: {e}')
            return 'failure'
        finally:
            self.node.destroy_node()
    
    def left_callback(self, msg: Point):
        if not math.isnan(msg.x) and not math.isnan(msg.y):
            self.node.get_logger().info(f'Объект обнаружен в левой камере ({msg.x},{msg.y})')
            self.flag_left = True
        else:
            self.flag_left = False

    def right_callback(self, msg: Point):
        if not math.isnan(msg.x) and not math.isnan(msg.y):
            self.node.get_logger().info(f'Объект обнаружен в правой камере ({msg.x},{msg.y})')
            self.flag_right = True
        else:
            self.flag_right = False

    def delay_with_callbacks(self, node, delay_seconds):
        """Задержка с обработкой колбэков"""
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < delay_seconds:
            # Обрабатываем колбэки в течение короткого таймаута
            rclpy.spin_once(node, timeout_sec=0.1)



class AlignmentState(smach.State):
    def __init__(self, target_object):
        smach.State.__init__(self, outcomes=['aligned', 'object_lost', 'failure'])
        self.target_object = target_object
        self.turn_left_flag = False
        self.turn_right_flag = False
        self.stop_flag = False
        self.published_left = False
        self.published_right = False
        self.counter_left = 0
        self.counter_right = 0
        self.flag_object_lost = False
        
    def execute(self, userdata):
        # Создаем временный узел
        node = rclpy.create_node('alignment_state')
        
        try:
            node.get_logger().info(f'=== ВЫРАВНИВАНИЕ С ОБЪЕКТОМ: {self.target_object} ===')
            
            # Создаем издателя для точного позиционирования
            cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)

            name_left = ''
            name_right = ''

            if self.target_object == 'blue_cyl':
                name_left = 'left/blue_cyl_center'
                name_right = 'right/blue_cyl_center'
            elif self.target_object == 'green_cyl':
                name_left = 'left/green_cyl_center'
                name_right = 'right/green_cyl_center'

            # Подписка на топики с изображениями левой и правой камер
            self.left_subscription = node.create_subscription(
                Point,
                name_left,
                self.left_callback,
                10
            )

            self.right_subscription = node.create_subscription(
                Point,
                name_right,
                self.right_callback,
                10
            )

            self.triangulator_sub = node.create_subscription(
            PointStamped,
            '/triangulated_object_center',
            self.triangulator_callback,
            10
        )
            
            time.sleep(0.5)

            while not self.stop_flag and not self.flag_object_lost:
                if self.turn_left_flag:
                    if not self.published_left:
                        rotate_msg = Twist()
                        rotate_msg.angular.z = -0.05
                        cmd_vel_publisher.publish(rotate_msg)
                        self.published_left = True
                        self.published_right = False
                elif self.turn_right_flag:
                    if not self.published_right:
                        rotate_msg = Twist()
                        rotate_msg.angular.z = 0.05
                        cmd_vel_publisher.publish(rotate_msg)
                        self.published_right = True
                        self.published_left = False
                
                self.delay_with_callbacks(node=node, delay_seconds=0.1)
            
            # Останавливаемся
            stop_msg = Twist()
            cmd_vel_publisher.publish(stop_msg)
            self.delay_with_callbacks(node=node, delay_seconds=1)

            if self.flag_object_lost:
                node.get_logger().warn(f'Объект {self.target_object} потерян ищу занова')
                return 'object_lost'
            
            node.get_logger().info(f'Выравнивание с {self.target_object} завершено')
            return 'aligned'
            
        except Exception as e:
            node.get_logger().error(f'Ошибка в состоянии выравнивания: {e}')
            return 'failure'
        finally:
            node.destroy_node()

    def triangulator_callback(self, msg: PointStamped):
        if not math.isnan(msg.point.x) and not math.isnan(msg.point.y) and not math.isnan(msg.point.z):
            if abs(msg.point.y) < 0.03:
                self.stop_flag = True
            
            if msg.point.y < 0:
                self.turn_right_flag = True
                self.turn_left_flag = False
            else:
                self.turn_left_flag = True
                self.turn_right_flag = False

    def left_callback(self, msg: Point):
        if math.isnan(msg.x) or math.isnan(msg.y):
            self.counter_left += 1
        else:
            self.counter_left = 0

        if self.counter_left > 10:
            self.flag_object_lost = True
            

    def right_callback(self, msg: Point):
        if math.isnan(msg.x) or math.isnan(msg.y):
            self.counter_right += 1
        else:
            self.counter_right = 0

        if self.counter_right > 10:
            self.flag_object_lost = True

    def delay_with_callbacks(self, node, delay_seconds):
        """Задержка с обработкой колбэков"""
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < delay_seconds:
            # Обрабатываем колбэки в течение короткого таймаута
            rclpy.spin_once(node, timeout_sec=0.1)


class ClosingState(smach.State):
    def __init__(self, target_object):
        smach.State.__init__(self, outcomes=['closed', 'object_lost', 'failure'],output_keys=['target_point'])
        self.target_object = target_object
        self.node = None
        self.flag_object_lost = False
        self.counter = 0
        self.flag_stop = False
        self.transformed_points_queue = []
        self.thread = None
        self.lock = threading.Lock()
        self.point_from_trng = None
        self.point_updated = False
        
    def execute(self, userdata):
        self.node = rclpy.create_node('closing_state')
        
        try:
            self.node.get_logger().info(f'=== СБЛИЖЕНИЕ С ОБЪЕКТОМ: {self.target_object} ===')

            self.client = self.node.create_client(CoordinateTransform, 'coordinate_transform')
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('Service not available, waiting again...')
            
            cmd_vel_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

            self.triangulator_sub = self.node.create_subscription(
                PointStamped,
                '/triangulated_object_center',
                self.triangulator_callback,
                10
            )
            
            time.sleep(0.5)
            twist_msg = Twist()
            twist_msg.linear.x = 0.1
            cmd_vel_publisher.publish(twist_msg)

            # Главный цикл
            self.thread = threading.Thread(target=self.transform_point_thread,daemon=True)
            self.thread.start()

            while rclpy.ok() and not self.flag_stop and not self.flag_object_lost:
                # Проверяем преобразованные точки
                with self.lock:
                    if self.transformed_points_queue:
                        point = self.transformed_points_queue.pop(0)
                        if point.x < 0.4:
                            self.flag_stop = True
                            self.node.get_logger().info(f'Достигнуто целевое расстояние: {point.x}')
                
                self.delay_with_callbacks(node=self.node,delay_seconds=0.1)
            
            stop_msg = Twist()
            cmd_vel_publisher.publish(stop_msg)
            self.delay_with_callbacks(node=self.node,delay_seconds=1)

            # После остановки и задержки в секундку снова смотрим какая целевая точка получилась (если этого не сделать, то точка может быть смещена вперед)
            with self.lock:
                point = self.transformed_points_queue[-1]
                userdata.target_point = point
                self.node.get_logger().info(f'Целевая точка захвата: {point}')

            if self.flag_object_lost:
                return 'object_lost'
            
            return 'closed'
            
        except Exception as e:
            self.node.get_logger().error(f'Ошибка: {e}')
            return 'failure'
        finally:
            self.node.destroy_node()

    def triangulator_callback(self, msg: PointStamped):
        if math.isnan(msg.point.x) or math.isnan(msg.point.y) or math.isnan(msg.point.z):
            self.counter += 1
        else:
            self.counter = 0
            with self.lock:
                self.point_from_trng = msg.point
                self.point_updated = True

        if self.counter > 10:
            self.flag_object_lost = True

    def delay_with_callbacks(self, node, delay_seconds):
        """Задержка с обработкой колбэков"""
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < delay_seconds:
            # Обрабатываем колбэки в течение короткого таймаута
            rclpy.spin_once(node, timeout_sec=0.1)

    def transform_point_thread(self):
        """Преобразование координат в отдельном потоке"""
        point = None
        updated = False
        while True:
            with self.lock:
                if self.point_updated:
                    self.point_updated = False
                    updated = True
                    point = self.point_from_trng
            
            if updated:
                updated = False
                try:
                    request = CoordinateTransform.Request()
                    request.point = point
                    request.source_frame = 'base_link'
                    request.target_frame = 'arm_link'
                    
                    future = self.client.call_async(request)
                    rclpy.spin_until_future_complete(self.node, future)
                    
                    if future.result() is not None:
                        transformed_point = future.result().transformed_point
                        with self.lock:
                            self.transformed_points_queue.append(transformed_point)
                    else:
                        self.node.get_logger().error('Ошибка преобразования')
                except Exception as e:
                    self.node.get_logger().error(f'Ошибка в потоке преобразования: {e}')

            time.sleep(0.1)

class ManipMoveWork(smach.State):
    def __init__(self, target_object):
        smach.State.__init__(self, outcomes=['moved_work', 'failure'], input_keys=['target_point'])
        
        self.node = None
        self.counter = 0
        self.current_point = Point()
        
    def execute(self, userdata):
        self.node = rclpy.create_node('manip_moving_working_state')

        target_point = userdata.target_point
        
        try:
            self.node.get_logger().info(f'=== ДВИЖЕНИЕ МАНИПУЛЯТОРА В РАБОЧЕЕ ПОЛОЖЕНИЕ ДЛЯ ЗАХВАТА: {target_point} ===')

            arm_target_point_pub = self.node.create_publisher(Pose, '/arm_target_pose', 10)

            self.arm_current_point_sub = self.node.create_subscription(
                Point,
                '/arm_current_point',
                self.arm_current_point_callback,
                10
            )
            
            # Задержка с целью того, чтобы пришла текущая точка манипулятора
            i = 0
            while i < 5:
                self.delay_with_callbacks(node=self.node, delay_seconds=0.5)
                i+=1

            # Планирование траектории
            preGrapPoint = Pose()
            preGrapPoint.position.x = 0.3
            preGrapPoint.position.y = 0.0
            preGrapPoint.position.z = 0.3

            preGrapPoint.orientation.x = 0.0
            preGrapPoint.orientation.y = math.sqrt(2)/2
            preGrapPoint.orientation.z = 0.0
            preGrapPoint.orientation.w = math.sqrt(2)/2
            arm_target_point_pub.publish(preGrapPoint)

            self.node.get_logger().info(f'Рабочее положение: {preGrapPoint}')

            while self.counter < 20 and (self.current_point.x - preGrapPoint.position.x > 0.01 or 
                                         self.current_point.y - preGrapPoint.position.y > 0.01 or 
                                         self.current_point.z - preGrapPoint.position.z > 0.01):
                self.node.get_logger().info(f'Текущая точка манипулятора: {self.current_point}')
                self.delay_with_callbacks(node=self.node, delay_seconds=0.5)
                arm_target_point_pub.publish(preGrapPoint)
                self.counter += 1

            if self.counter >= 20:
                self.node.get_logger().error(f'Манипулятор не сдвинулся в течение 5 секунд')
                return 'failure'
            
            return 'moved_work'

            
        except Exception as e:
            self.node.get_logger().error(f'Ошибка: {e}')
            return 'failure'
        finally:
            self.node.destroy_node()

    def arm_current_point_callback(self, msg: Point):
        self.current_point.x = msg.x
        self.current_point.y = msg.y
        self.current_point.z = msg.z


    def delay_with_callbacks(self, node, delay_seconds):
        """Задержка с обработкой колбэков"""
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < delay_seconds:
            # Обрабатываем колбэки в течение короткого таймаута
            rclpy.spin_once(node, timeout_sec=0.1)


class ManipMovingState(smach.State):
    def __init__(self, target_object):
        smach.State.__init__(self, outcomes=['moved', 'failure'], input_keys=['target_point'])
        
        self.node = None
        self.counter = 0
        self.current_point = Point()
        
    def execute(self, userdata):
        self.node = rclpy.create_node('manip_moving_state')

        target_point = userdata.target_point
        
        try:
            self.node.get_logger().info(f'=== ДВИЖЕНИЕ МАНИПУЛЯТОРА В ТОЧКЕ: {target_point} ===')

            gripper_cap_pub = self.node.create_publisher(Float64, '/gripper_target_gap', 10)

            arm_target_point_pub = self.node.create_publisher(Pose, '/arm_target_pose', 10)

            self.arm_current_point_sub = self.node.create_subscription(
                Point,
                '/arm_current_point',
                self.arm_current_point_callback,
                10
            )
            
            # Задержка с целью того, чтобы пришла текущая точка манипулятора
            i = 0
            while i < 5:
                self.delay_with_callbacks(node=self.node, delay_seconds=0.5)
                i+=1

            # Открываем ЗУ
            f = Float64()
            f.data = 0.12
            gripper_cap_pub.publish(f)

            # Планирование траектории
            preGrapPoint = Pose()
            preGrapPoint.position.x = self.current_point.x + (target_point.x - self.current_point.x)*0.75
            preGrapPoint.position.y = self.current_point.y + (target_point.y - self.current_point.y)*0.75
            preGrapPoint.position.z = self.current_point.z + (target_point.z - self.current_point.z)*0.75

            # Вычисляем ориентацию схвата
            rotation_matrix = self.compute_target_orientation([preGrapPoint.position.x, preGrapPoint.position.y, preGrapPoint.position.z], -np.pi/6)
            quat = self.rotation_matrix_to_quaternion(rotation_matrix)

            preGrapPoint.orientation.x = quat[0]
            preGrapPoint.orientation.y = quat[1]
            preGrapPoint.orientation.z = quat[2]
            preGrapPoint.orientation.w = quat[3]
            arm_target_point_pub.publish(preGrapPoint)

            self.node.get_logger().info(f'Точка предзахвата: {preGrapPoint}')

            while self.counter < 20 and (self.current_point.x - preGrapPoint.position.x > 0.01 or 
                                         self.current_point.y - preGrapPoint.position.y > 0.01 or 
                                         self.current_point.z - preGrapPoint.position.z > 0.01):
                self.node.get_logger().info(f'Текущая точка манипулятора: {self.current_point}')
                self.delay_with_callbacks(node=self.node, delay_seconds=0.5)
                arm_target_point_pub.publish(preGrapPoint)
                self.counter += 1

            if self.counter >= 20:
                self.node.get_logger().error(f'Манипулятор не сдвинулся в течение 5 секунд')
                return 'failure'

            target_point_msg = Pose()
            target_point_msg.position.x = target_point.x
            target_point_msg.position.y = target_point.y
            target_point_msg.position.z = target_point.z

            rotation_matrix = self.compute_target_orientation([target_point_msg.position.x, 
                                                                               target_point_msg.position.y, target_point_msg.position.z], 
                                                                               -np.pi/6)
            quat = self.rotation_matrix_to_quaternion(rotation_matrix)

            target_point_msg.orientation.x = quat[0]
            target_point_msg.orientation.y = quat[1]
            target_point_msg.orientation.z = quat[2]
            target_point_msg.orientation.w = quat[3]

            arm_target_point_pub.publish(target_point_msg)

            self.node.get_logger().info(f'Точка захвата: {target_point_msg}')

            self.counter = 0
            while self.counter < 20 and (self.current_point.x - target_point.x > 0.01 or 
                                         self.current_point.y - target_point.y > 0.01 or 
                                         self.current_point.z - target_point.z > 0.01):
                self.node.get_logger().info(f'Текущая точка манипулятора: {self.current_point}')
                self.delay_with_callbacks(node=self.node, delay_seconds=0.5)
                arm_target_point_pub.publish(target_point_msg)
                self.counter += 1

            if self.counter >= 20:
                self.node.get_logger().error(f'Манипулятор не сдвинулся в течение 5 секунд')
                return 'failure'
            
            # Закрываем ЗУ
            f = Float64()
            f.data = 0.025
            gripper_cap_pub.publish(f)

            i = 0
            while i < 10:
                self.delay_with_callbacks(node=self.node, delay_seconds=0.2)
                i+=1

            target_point_msg = Pose()
            target_point_msg.position.x = 0.3
            target_point_msg.position.y = 0.0
            target_point_msg.position.z = 0.3

            rotation_matrix = self.compute_target_orientation([target_point_msg.position.x, 
                                                                               target_point_msg.position.y, target_point_msg.position.z], 
                                                                               -np.pi/6)
            quat = self.rotation_matrix_to_quaternion(rotation_matrix)

            target_point_msg.orientation.x = quat[0]
            target_point_msg.orientation.y = quat[1]
            target_point_msg.orientation.z = quat[2]
            target_point_msg.orientation.w = quat[3]

            arm_target_point_pub.publish(target_point_msg)

            self.node.get_logger().info(f'Точка подъёма захваченного объекта: {target_point_msg}')

            self.counter = 0
            while self.counter < 20 and (self.current_point.x - target_point.x > 0.01 or 
                                         self.current_point.y - target_point.y > 0.01 or 
                                         self.current_point.z - target_point.z > 0.01):
                self.node.get_logger().info(f'Текущая точка манипулятора: {self.current_point}')
                self.delay_with_callbacks(node=self.node, delay_seconds=0.5)
                arm_target_point_pub.publish(target_point_msg)
                self.counter += 1

            if self.counter >= 20:
                self.node.get_logger().error(f'Манипулятор не сдвинулся в течение 5 секунд')
                return 'failure'
            
            return 'moved'
            
        except Exception as e:
            self.node.get_logger().error(f'Ошибка: {e}')
            return 'failure'
        finally:
            self.node.destroy_node()

    def arm_current_point_callback(self, msg: Point):
        self.current_point.x = msg.x
        self.current_point.y = msg.y
        self.current_point.z = msg.z


    def delay_with_callbacks(self, node, delay_seconds):
        """Задержка с обработкой колбэков"""
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < delay_seconds:
            # Обрабатываем колбэки в течение короткого таймаута
            rclpy.spin_once(node, timeout_sec=0.1)

    def compute_target_orientation(self, target_position, twist_angle=0.0):
        
        # Новая ось Z направлена от нуля мировой СК к целевой точке
        z_axis = np.array(target_position)
        
        # Нормализуем ось Z
        if np.linalg.norm(z_axis) < 1e-6:
            # Если цель в начале координат, используем ось Z по умолчанию
            z_axis = np.array([0.0, 0.0, 1.0])
        else:
            z_axis = z_axis / np.linalg.norm(z_axis)
        
        # Новая ось X сонаправлена с мировой осью Z [0, 0, 1]
        x_axis = np.array([0.0, 0.0, 1.0])
        
        # Если ось Z почти параллельна мировой Z (вертикальна), 
        # то нужно выбрать другое направление для X
        if abs(np.dot(z_axis, x_axis)) > 0.99:
            # Если цель почти на вертикальной оси, используем мировую ось X для оси Y
            x_axis = np.array([1.0, 0.0, 0.0])
        else:
            # В общем случае ось X = мировая ось Z
            x_axis = np.array([0.0, 0.0, 1.0])
        
        # Ось Y = Z × X (правило правой тройки)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # Пересчитываем ось X = Y × Z для точной ортогональности
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Создаем базовую матрицу поворота
        rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
        
        # Применяем поворот вокруг новой оси Y
        if abs(twist_angle) > 1e-6:
            # Матрица поворота вокруг локальной оси Y
            cos_a = np.cos(twist_angle)
            sin_a = np.sin(twist_angle)
            Ry = np.array([
                [cos_a, 0, sin_a],
                [0, 1, 0],
                [-sin_a, 0, cos_a]
            ])
            
            # Применяем поворот к текущей ориентации
            rotation_matrix = rotation_matrix @ Ry
        
        return rotation_matrix
    
    def rotation_matrix_to_quaternion(self, rotation_matrix):

        # Убедимся, что матрица ортонормированная
        if abs(np.linalg.det(rotation_matrix) - 1.0) > 1e-6:
            U, _, Vt = np.linalg.svd(rotation_matrix)
            rotation_matrix = U @ Vt
        
        m = rotation_matrix
        trace = m[0, 0] + m[1, 1] + m[2, 2]
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (m[2, 1] - m[1, 2]) / S
            y = (m[0, 2] - m[2, 0]) / S
            z = (m[1, 0] - m[0, 1]) / S
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            w = (m[2, 1] - m[1, 2]) / S
            x = 0.25 * S
            y = (m[0, 1] + m[1, 0]) / S
            z = (m[0, 2] + m[2, 0]) / S
        elif m[1, 1] > m[2, 2]:
            S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            w = (m[0, 2] - m[2, 0]) / S
            x = (m[0, 1] + m[1, 0]) / S
            y = 0.25 * S
            z = (m[1, 2] + m[2, 1]) / S
        else:
            S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            w = (m[1, 0] - m[0, 1]) / S
            x = (m[0, 2] + m[2, 0]) / S
            y = (m[1, 2] + m[2, 1]) / S
            z = 0.25 * S
        
        return np.array([x, y, z, w])




class FinishState(smach.State):
    def __init__(self, target_object):
        smach.State.__init__(self, outcomes=['completed'])
        self.target_object = target_object
        
    def execute(self, userdata):
        # Создаем временный узел
        node = rclpy.create_node('finish_state')
        
        try:
            node.get_logger().info('=== ЗАДАЧА ВЫПОЛНЕНА ===')
            node.get_logger().info(f'Объект {self.target_object} успешно захвачен!')
            
            # Можно опубликовать финальное сообщение или выполнить другие действия
            return 'completed'
            
        finally:
            node.destroy_node()


def main():
    # Обработка аргументов командной строки
    if len(sys.argv) < 2:
        return
    
    target_object = sys.argv[1]
    valid_objects = ['blue_cyl', 'green_cyl']
    
    if target_object not in valid_objects:
        print(f"Ошибка: неизвестный объект '{target_object}'")
        print("Доступные объекты:", ', '.join(valid_objects))
        return
    
    rclpy.init()
    
    # Создаем конечный автомат
    sm = smach.StateMachine(outcomes=['task_completed', 'task_failed'])

    sm.userdata.point_data = None
    
    with sm:
        # Добавляем состояния, передавая только target_object
        smach.StateMachine.add('INITIAL', InitialState(target_object),
                               transitions={'success': 'SEARCHING',
                                           'failure': 'task_failed'})
        
        smach.StateMachine.add('SEARCHING', SearchingState(target_object),
                               transitions={'object_found': 'ALIGNMENT',
                                           'failure': 'task_failed'})
        
        smach.StateMachine.add('ALIGNMENT', AlignmentState(target_object),
                               transitions={'aligned': 'CLOSING',
                                           'object_lost': 'SEARCHING',
                                           'failure': 'task_failed'})
        
        smach.StateMachine.add('CLOSING', ClosingState(target_object),
                               transitions={'closed': 'MANIPMOVINGWORK',
                                           'object_lost': 'SEARCHING',
                                           'failure': 'task_failed'},
                                remapping={'target_point': 'point_data'})
        
        smach.StateMachine.add('MANIPMOVINGWORK', ManipMoveWork(target_object),
                               transitions={'moved_work': 'MANIPMOVING',
                                           'failure': 'MANIPMOVINGWORK'},
                                remapping={'target_point': 'point_data'})
        
        smach.StateMachine.add('MANIPMOVING', ManipMovingState(target_object),
                               transitions={'moved': 'FINISH',
                                           'failure': 'MANIPMOVING'},
                                remapping={'target_point': 'point_data'})
        
        smach.StateMachine.add('FINISH', FinishState(target_object),
                               transitions={'completed': 'task_completed'})
    
    # Создаем introspection сервер для визуализации в rqt
    sis = smach_ros.IntrospectionServer('youbot_state_machine', sm, '/YOUROT_SM')
    sis.start()
    
    # Создаем временный узел для основного логгера
    main_node = rclpy.create_node('state_machine_main')
    main_node.get_logger().info(f'Запуск конечного автомата YouBot для объекта: {target_object}')
    main_node.get_logger().info('Для визуализации запустите: rqt')
    
    try:
        # Запускаем выполнение автомата
        outcome = sm.execute()
        main_node.get_logger().info(f'Конечный автомат завершил работу с результатом: {outcome}')
        
    except KeyboardInterrupt:
        main_node.get_logger().info('Получен сигнал прерывания')
    except Exception as e:
        main_node.get_logger().error(f'Ошибка в конечном автомате: {e}')
    finally:
        # Всегда останавливаем introspection сервер
        sis.stop()
        main_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
