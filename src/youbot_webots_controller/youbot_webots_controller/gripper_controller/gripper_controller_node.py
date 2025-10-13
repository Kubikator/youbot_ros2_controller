#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        self.get_logger().info('Starting Gripper Controller...')
        
        # Параметры схвата (взято из gripper.c)
        self.MIN_POS = 0.0
        self.MAX_POS = 0.025
        self.OFFSET_WHEN_LOCKED = 0.021
        self.MAX_GAP = 2 * self.MAX_POS + self.OFFSET_WHEN_LOCKED
        self.MIN_GAP = 2 * self.MIN_POS + self.OFFSET_WHEN_LOCKED
        
        # Текущее состояние
        self.current_gap = self.MAX_GAP
        self.target_gap = self.MAX_GAP
        
        # Публикаторы для управления схватом
        self.gripper_gap_pub = self.create_publisher(
            Float64,
            'gripper_current_gap',
            10
        )

        self.gripper_pos_pub = self.create_publisher(
            Float64,
            'gripper_position',
            10
        )
        
        # Подписчики
        self.create_subscription(Float64, 'gripper_current_position',
                                self.gripper_current_position_callback, 10)
        
        self.create_subscription(Float64, 'gripper_target_gap',
                                self.target_gap_callback, 10)
        
        self.get_logger().info('Gripper Controller initialized successfully')
        self.get_logger().info(f'Gripper gap range: {self.MIN_GAP:.3f} - {self.MAX_GAP:.3f} m')

    def target_gap_callback(self, msg):
        target_gap = max(min(msg.data, self.MAX_GAP), self.MIN_GAP)
        self.get_logger().info(f'Received target gap: {target_gap}')

         # Преобразуем зазор в позицию как в оригинальном коде
        v = max(min(0.5 * (target_gap - self.OFFSET_WHEN_LOCKED), self.MAX_POS), self.MIN_POS)
        pos_msg = Float64()
        pos_msg.data = v
        self.gripper_pos_pub.publish(pos_msg)

    def gripper_current_position_callback(self, msg):
        position = max(min(msg.data, self.MAX_POS), self.MIN_POS)
        self.current_gap = 2 * position + self.OFFSET_WHEN_LOCKED

        gap_msg = Float64()
        gap_msg.data = self.current_gap
        self.gripper_gap_pub.publish(gap_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = GripperController()
        rclpy.spin(controller)
    except Exception as e:
        print(f"Failed to start Gripper Controller: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()