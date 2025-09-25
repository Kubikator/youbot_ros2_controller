import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtCore import QTimer, QObject


class QNode(QObject):
    
    def __init__(self, node_name='QNode'):
        super().__init__()
        
        if not rclpy.ok():
            rclpy.init()
            
        self.node = Node(node_name)
        
        self.cmd_vel_publisher = self.node.create_publisher(
            Twist, 'cmd_vel', 10
        )
        
        # Таймер для обработки ROS2 callbacks
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_once)
        self.ros_timer.start(10)
        
        self.get_logger().info(f"'{node_name}' succesfully initialized")
        
    def get_logger(self):
        return self.node.get_logger()
        
    def spin_once(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.001)
        except Exception as e:
            self.get_logger().error(f"Error in spin_once: {e}")
            
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
