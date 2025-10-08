#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from youbot_webots_controller.youbot_arm_controller.arm_kinematic_solver import KukaYouBotKinematic

class ArmKinematicsSolver(Node):
    def __init__(self):
        super().__init__('arm_kinematics_solver')
        
        self.get_logger().info('Starting Arm Kinematics Solver...')
        
        # Инициализация класса кинематики
        self.kinematics = KukaYouBotKinematic()
        
        # Параметры манипулятора YouBot
        self.arm_joint_names = [
            'arm1',
            'arm2',
            'arm3',
            'arm4',
            'arm5' 
        ]
        
        # Пределы углов для каждого сустава
        self.joint_limits = {
            'arm1': (-2.95, 2.95),
            'arm2': (-1.13, 1.13),
            'arm3': (-2.07, 2.07),
            'arm4': (-1.57, 1.57),
            'arm5': (-2.07, 2.07)
        }
        
        # Текущие позиции суставов
        self.current_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Публикатор для целевых позиций
        self.arm_target_pub = self.create_publisher(
            Float64MultiArray,
            'arm_target_positions',
            10
        )

        self.arm_current_pos = self.create_publisher(
            Point,
            'arm_current_point',
            10
        )
        
        # Подписчики
        self.create_subscription(JointState, 'arm_joints_states',
                                self.joint_states_callback, 10)
        self.create_subscription(Float64MultiArray, 'arm_target_joints',
                                self.target_joints_callback, 10)
        self.create_subscription(Pose, 'arm_target_pose',
                                self.target_pose_callback, 10)
        
        self.get_logger().info('Arm Kinematics Solver initialized successfully')

    def joint_states_callback(self, msg):
        try:
            if set(msg.name) == set(self.arm_joint_names):
                for i, joint_name in enumerate(self.arm_joint_names):
                    if joint_name in msg.name:
                        idx = msg.name.index(joint_name)
                        self.current_positions[i] = msg.position[idx]
                        T = (self.kinematics.forward_kinematic(joint_angles=self.current_positions))
                        point = T[:3,3]
                        msg_point = Point()
                        msg_point.x = point[0]
                        msg_point.y = point[1]
                        msg_point.z = point[2]
                        self.arm_current_pos.publish(msg_point)
        except Exception as e:
            self.get_logger().error(f'Error in joint_states_callback: {e}')

    def target_joints_callback(self, msg):
        """Получение обобщенных координат (углов суставов)"""
        self.get_logger().info(f'Received target joint positions: {msg.data}')
        
        if len(msg.data) == 5:
            # Проверяем и ограничиваем позиции
            target_positions = []
            for i, (pos, name) in enumerate(zip(msg.data, self.arm_joint_names)):
                limited_pos = max(min(pos, self.joint_limits[name][1]), self.joint_limits[name][0])
                target_positions.append(limited_pos)
            
            self.send_target_positions(target_positions)
        else:
            self.get_logger().warn(f'Invalid number of positions: {len(msg.data)} (should be 5)')

    def target_pose_callback(self, msg):
        """Получение целевой позы (x, y, z и ориентация)"""
        try:
            # Извлекаем позицию
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            
            # Извлекаем pitch из кватерниона ориентации
            # Для упрощения используем только pitch (наклон вперед-назад)
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w

            rot_mat = self.quaternion_to_rotation_matrix([qx, qy, qz, qw])
            
            self.get_logger().info(f'Received target pose: x={x:.3f}, y={y:.3f}, z={z:.3f}')
            
            # Решение обратной кинематики
            target_pos = np.array([x, y, z])

            target_orient = self.compute_target_orientation(target_pos, -np.pi/4)

            self.get_logger().info(f'Target orientation: {target_orient}')

            target_pose = np.eye(4)
            target_pose[:3,:3] = target_orient
            target_pose[:3,3] = target_pos
            #joint_angles, success, error = self.kinematics.inverse_kinematic_multistart(target_pose)
            joint_angles, success, error = self.kinematics.inverse_kinematic_trust_constr(target_pose=target_pose)

            if not success:
                self.get_logger().info(f'Численный метод не сошелся. Ошибка: {error}')
            
            # Проверяем и ограничиваем углы
            target_positions = []
            for i, (angle, name) in enumerate(zip(joint_angles, self.arm_joint_names)):
                limited_angle = max(min(angle, self.joint_limits[name][1]), self.joint_limits[name][0])
                target_positions.append(limited_angle)
            
            self.get_logger().info(f'Computed joint angles: {target_positions}')
            self.send_target_positions(target_positions)
            
        except Exception as e:
            self.get_logger().error(f'Error in target_pose_callback: {e}')

    def send_target_positions(self, positions):
        try:
            target_msg = Float64MultiArray()
            target_msg.data = positions
            self.arm_target_pub.publish(target_msg)
            self.get_logger().info(f'Sent target positions: {positions}')
        except Exception as e:
            self.get_logger().error(f'Error sending target positions: {e}')

    def quaternion_to_rotation_matrix(self, quaternion):
        """
        Преобразует кватернион [x, y, z, w] в матрицу поворота 3x3
        """
        x, y, z, w = quaternion
        
        # Нормализуем кватернион
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Вычисляем матрицу поворота
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        
        rotation_matrix = np.array([
            [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
            [2*(xy + wz),     1 - 2*(xx + zz),     2*(yz - wx)],
            [2*(xz - wy),         2*(yz + wx),     1 - 2*(xx + yy)]
        ])
        
        return rotation_matrix
    
    def compute_target_orientation(self, target_position, twist_angle=0.0):
        
        # Новая ось Z направлена от нуля мировой СК к целевой точке
        z_axis = np.array(target_position)
        z_axis[2] = 0.0
        
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


def main(args=None):
    rclpy.init(args=args)
    
    try:
        kinematics_calculator = ArmKinematicsSolver()
        rclpy.spin(kinematics_calculator)
    except Exception as e:
        print(f"Failed to start Arm Kinematics Solver: {e}")
    finally:
        kinematics_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()