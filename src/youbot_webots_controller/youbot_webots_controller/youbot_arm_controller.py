#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
import time
from threading import Thread

class ArmKinematicsSolver(Node):
    def __init__(self):
        super().__init__('arm_kinematics_solver')
        
        self.get_logger().info('Starting Arm Kinematics Solver...')
        
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
        self.target_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.is_moving = False
        
        # Публикатор для целевых позиций
        self.arm_target_pub = self.create_publisher(
            Float64MultiArray,
            'arm_target_positions',
            10
        )
        
        # Подписчики
        self.create_subscription(Float64MultiArray, 'arm_joints_positions', 
                                self.joints_positions_callback, 10)
        self.create_subscription(JointTrajectory, 'arm_joints_trajectory',
                                self.joint_trajectory_callback, 10)
        self.create_subscription(JointState, 'arm_joints_states',
                                self.joint_states_callback, 10)
        
        self.get_logger().info('Arm Kinematics Solver initialized successfully')

    def joint_states_callback(self, msg):
        try:
            if set(msg.name) == set(self.arm_joint_names):
                for i, joint_name in enumerate(self.arm_joint_names):
                    if joint_name in msg.name:
                        idx = msg.name.index(joint_name)
                        self.current_positions[i] = msg.position[idx]
        except Exception as e:
            self.get_logger().error(f'Error in joint_states_callback: {e}')

    def joints_positions_callback(self, msg):
        self.get_logger().info(f'Received joints target positions: {msg.data}')
        
        if len(msg.data) == 5:
            # Проверяем и ограничиваем позиции
            target_positions = []
            for i, (pos, name) in enumerate(zip(msg.data, self.arm_joint_names)):
                limited_pos = max(min(pos, self.joint_limits[name][1]), self.joint_limits[name][0])
                target_positions.append(limited_pos)
            
            self.send_target_positions(target_positions)
        else:
            self.get_logger().warn(f'Invalid number of positions: {len(msg.data)} (should be 5)')

    def joint_trajectory_callback(self, msg):
        self.get_logger().info(f'Received joint trajectory for: {msg.joint_names}')
        
        if set(msg.joint_names) == set(self.arm_joint_names):
            # Запускаем выполнение траектории в отдельном потоке
            Thread(target=self.execute_trajectory, args=(msg.points,), daemon=True).start()
        else:
            self.get_logger().warn('Joint names mismatch in trajectory')

    def send_target_positions(self, positions):
        try:
            target_msg = Float64MultiArray()
            target_msg.data = positions
            self.arm_target_pub.publish(target_msg)
            self.get_logger().info(f'Sent target positions: {positions}')
        except Exception as e:
            self.get_logger().error(f'Error sending target positions: {e}')

    def execute_trajectory(self, points):
        if self.is_moving:
            self.get_logger().info('Movement already in progress, waiting...')
            return
        
        self.is_moving = True
        
        try:
            for point_idx, point in enumerate(points):
                self.get_logger().info(f'Executing trajectory point {point_idx + 1}/{len(points)}')
                
                # Получаем целевые позиции для этой точки траектории
                target_positions = [
                    max(min(pos, self.joint_limits[name][1]), self.joint_limits[name][0])
                    for pos, name in zip(point.positions, self.arm_joint_names)
                ]
                
                # Отправляем позиции
                self.send_target_positions(target_positions)
                
                # Ждем указанное время перед переходом к следующей точке
                wait_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                time.sleep(wait_time)
            
            self.get_logger().info('Trajectory execution completed')
            
        except Exception as e:
            self.get_logger().error(f'Error executing trajectory: {e}')
        finally:
            self.is_moving = False

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