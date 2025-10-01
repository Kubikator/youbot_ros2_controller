import sys
import os
import rclpy
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from PyQt5 import uic

from .qnode import QNode

class PlatformControllerGui(QWidget):

    def __init__(self, ui_file_path=None):
        super().__init__()
        
        # Создание ROS2 ноды
        self.qnode = QNode('QNode')
        
        # Путь к UI файлу
        self.ui_file_path = ui_file_path or 'UI_PlatformCTL.ui'
        
        # Инициализация интерфейса
        self.init_ui()

        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        # Подключение сигналов
        self.setup_connections()
        
        self.qnode.get_logger().info("PlatformControllerGui has been initialized")
        
    def init_ui(self):
        try:
            # Загрузка UI файла из Qt Designer
            uic.loadUi(self.ui_file_path, self)
            self.qnode.get_logger().info(f"UI loaded from file: {self.ui_file_path}")
            
            # Настройка QLabel для отображения изображений
            if hasattr(self, 'display'):
                self.display.setAlignment(Qt.AlignCenter)
                self.display.setScaledContents(True)  # Включаем автомасштабирование
                #self.display.setMinimumSize(640, 480)
                self.qnode.get_logger().info("QLabel 'display' configured for image display")
            
        except Exception as e:
            self.qnode.get_logger().error(f"Failed to initialize UI: {e}")
        
    def setup_connections(self):
        try:
            # Подключаем сигнал изображения от QNode
            self.qnode.image_received.connect(self.update_image_display)
            
            # Кнопки движения (нажатие/отпускание для непрерывного движения)
            if hasattr(self, 'b_forward'):
                self.qnode.get_logger().info("b_forward connecting to QNode...")
                self.b_forward.pressed.connect(lambda: self.qnode.publish_velocity(linear_x=self.linear_speed))
                self.b_forward.released.connect(self.qnode.stop_robot)
                self.qnode.get_logger().info("b_forward connected to QNode")

            if hasattr(self, 'b_reverse'):
                self.qnode.get_logger().info("b_reverse connecting to QNode...")
                self.b_reverse.pressed.connect(lambda: self.qnode.publish_velocity(linear_x= -self.linear_speed))
                self.b_reverse.released.connect(self.qnode.stop_robot)
                self.qnode.get_logger().info("b_reverse connected to QNode")
                
            if hasattr(self, 'b_counterClockwise'):
                self.qnode.get_logger().info("b_counterClockwise connecting to QNode...")
                self.b_counterClockwise.pressed.connect(lambda: self.qnode.publish_velocity(angular_z=-self.angular_speed))
                self.b_counterClockwise.released.connect(self.qnode.stop_robot)
                self.qnode.get_logger().info("b_counterClockwise connected to QNode")
                
            if hasattr(self, 'b_clockwise'):
                self.qnode.get_logger().info("b_clockwise connected to QNode...")
                self.b_clockwise.pressed.connect(lambda: self.qnode.publish_velocity(angular_z=  self.angular_speed))
                self.b_clockwise.released.connect(self.qnode.stop_robot)
                self.qnode.get_logger().info("b_clockwise connected to QNode")
                
            self.qnode.get_logger().info("All signals connected to QNode")
            
        except Exception as e:
            self.qnode.get_logger().error(f"Failed to connect signals to QNode: {e}")
    
    def update_image_display(self, qimage):
        """Обновление QLabel с изображением (вызывается через сигнал)"""
        try:
            if hasattr(self, 'display'):
                # Конвертируем QImage в QPixmap и устанавливаем в QLabel
                pixmap = QPixmap.fromImage(qimage)
                self.display.setPixmap(pixmap)
                
        except Exception as e:
            self.qnode.get_logger().error(f"Error updating image display: {e}")
        
    def closeEvent(self, event):
        try:
            self.qnode.get_logger().info("Closing GUI...")
            self.qnode.shutdown()
            event.accept()
        except Exception as e:
            print(f"Error since close GUI: {e}")
            event.accept()

def main():
    app = QApplication(sys.argv)
    
    # Путь к UI файлу
    from ament_index_python.packages import get_package_share_directory
    package_share_dir = get_package_share_directory('youbot_webots_controller')
    ui_file = os.path.join(package_share_dir, 'ui', 'UI_PlatformCTL.ui')
    
    # Создание примера UI файла, если он не существует
    if not os.path.exists(ui_file):
        print(f"UI file {ui_file} does not exist")
        sys.exit()
    
    try:
        # Создание и отображение GUI
        controller = PlatformControllerGui(ui_file)
        controller.show()
        
        sys.exit(app.exec_())
        
    except KeyboardInterrupt:
        print("\nProgramm stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Убедимся, что ROS2 корректно завершен
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()