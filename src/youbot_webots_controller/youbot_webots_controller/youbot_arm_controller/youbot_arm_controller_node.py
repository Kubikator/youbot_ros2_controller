#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from youbot_webots_controller.youbot_arm_controller.arm_kinematic_solver import KukaYouBotKinematics

class ArmKinematicsSolver(Node):
    def __init__(self):
        super().__init__('arm_kinematics_solver')
        
        self.get_logger().info('Starting Arm Kinematics Solver...')
        
        # Инициализация класса кинематики
        self.kinematics = KukaYouBotKinematics()
        
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
                        point = self.kinematics.forward_kinematics(joints=self.current_positions)
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
            
            # Преобразование кватерниона в углы Эйлера (pitch)
            pitch = np.arcsin(2.0 * (qw * qy - qz * qx))
            
            self.get_logger().info(f'Received target pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, pitch={pitch:.3f}')
            
            # Решение обратной кинематики
            target_pos = np.array([x, y, z])
            joint_angles = self.kinematics.inverse_kinematics(target_pos, pitch)
            
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