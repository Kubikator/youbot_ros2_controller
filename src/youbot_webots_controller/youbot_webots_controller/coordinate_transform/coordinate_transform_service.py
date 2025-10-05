#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point

from youbot_webots_controller.srv import CoordinateTransform

class CoordinateTransformService(Node):
    def __init__(self):
        super().__init__('coordinate_transform_service')
        
        # Создаем сервис
        self.srv = self.create_service(
            CoordinateTransform, 
            'coordinate_transform', 
            self.transform_callback
        )
        
        # Инициализируем TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Coordinate Transform Service ready')

    async def transform_callback(self, request, response):
        """
        Callback функция для обработки запросов преобразования координат
        """
        try:
            self.get_logger().info(
                f'Received transform request: '
                f'point=({request.point.x}, {request.point.y}, {request.point.z}), '
                f'source_frame={request.source_frame}, '
                f'target_frame={request.target_frame}'
            )
            
            # Получаем трансформацию между системами координат
            transform = await self.tf_buffer.lookup_transform_async(
                request.target_frame,
                request.source_frame,
                rclpy.time.Time()
            )
            
            # Создаем PointStamped для преобразования
            from geometry_msgs.msg import PointStamped
            point_stamped = PointStamped()
            point_stamped.header.frame_id = request.source_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point = request.point
            
            # Применяем трансформацию
            transformed_point_stamped = do_transform_point(point_stamped, transform)
            
            # Заполняем ответ
            response.transformed_point = transformed_point_stamped.point
            
            self.get_logger().info(
                f'Transformed point: '
                f'({response.transformed_point.x}, {response.transformed_point.y}, {response.transformed_point.z})'
            )
            
            return response
            
        except TransformException as e:
            self.get_logger().error(f'Transform failed: {e}')
            # Возвращаем исходную точку в случае ошибки
            response.transformed_point = request.point
            return response
        except Exception as e:
            self.get_logger().error(f'Service error: {e}')
            response.transformed_point = request.point
            return response


def main():
    rclpy.init()
    
    coordinate_transform_service = CoordinateTransformService()
    
    try:
        rclpy.spin(coordinate_transform_service)
    except KeyboardInterrupt:
        pass
    finally:
        coordinate_transform_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()