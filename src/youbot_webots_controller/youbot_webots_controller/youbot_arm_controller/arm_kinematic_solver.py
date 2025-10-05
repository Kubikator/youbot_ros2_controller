import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class KukaYouBotKinematics:
    def __init__(self):
        # Параметры длин звеньев KUKA youBot
        self.l1 = 0.253  # ARM1 - высота основания
        self.l2 = 0.155  # ARM2
        self.l3 = 0.135  # ARM3
        self.l4 = 0.081  # ARM4
        self.l5 = 0.105  # ARM5
        
    def forward_kinematics(self, joints):
        """
        Прямая кинематика
        joints: массив из 5 углов суставов [j1, j2, j3, j4, j5] в радианах
        Возвращает: позицию конечного эффектора [x, y, z]
        """
        j1, j2, j3, j4, j5 = joints
        
        y1 = self.l2 * np.cos(j2) + self.l3 * np.cos(j2 + j3) + (self.l4 + self.l5) * np.cos(j2 + j3 + j4)
        z1 = self.l2 * np.sin(j2) + self.l3 * np.sin(j2 + j3) + (self.l4 + self.l5) * np.sin(j2 + j3 + j4)
        
        x = -y1 * np.sin(j1)
        y = y1 * np.cos(j1)
        z = self.l1 + z1
        
        return np.array([x, y, z])
    
    def inverse_kinematics(self, target_pos, pitch):
        """
        Обратная кинематика
        target_pos: целевая позиция [x, y, z]
        pitch: угол наклона конечного эффектора (в радианах)
        Возвращает: массив углов суставов [j1, j2, j3, j4, j5]
        """
        x, y, z = target_pos
        
        y1 = np.sqrt(x**2 + y**2)
        z1 = z + self.l4 + self.l5 - self.l1
        
        a = self.l2
        b = self.l3
        c = np.sqrt(y1**2 + z1**2)
        
        j1 = -np.arcsin(x / y1)
        j2 = -(np.pi/2 - np.arccos((a**2 + c**2 - b**2) / (2.0 * a * c)) - np.arctan2(z1, y1))
        j3 = -(np.pi - np.arccos((a**2 + b**2 - c**2) / (2.0 * a * b)))
        j4 = -(np.pi + (j2 + j3))
        j5 = 0.0
        #j5 = np.pi/2 + j1
        
        return np.array([j1, j2, j3, j4, j5])