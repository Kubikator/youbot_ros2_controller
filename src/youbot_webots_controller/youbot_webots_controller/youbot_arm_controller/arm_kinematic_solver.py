import numpy as np
from scipy.optimize import minimize, BFGS
import warnings

class KukaYouBotKinematic:
    def __init__(self):
        # Параметры Денавитта-Хартенберга для KUKA youBot (5 DOF)
        self.dh_params = [
            [0.033, np.pi/2, 0.253, 0],      # Joint 1
            [0.155, 0, 0, np.pi/2],          # Joint 2  
            [0.135, 0, 0, 0],                # Joint 3
            [0, np.pi/2, 0, np.pi/2],        # Joint 4
            [0, 0, 0.27, 0]                  # Joint 5
        ]
        
        # Ограничения на углы сочленений (в радианах)
        self.joint_limits = [
            (-2.96, 2.96),    # Joint 1
            (-1.57, 1.13),    # Joint 2
            (-2.09, 2.09),    # Joint 3
            (-1.92, 1.92),    # Joint 4
            (-2.96, 2.96)     # Joint 5
        ]
    
    def dh_matrix(self, a, alpha, d, theta):
        """Создает матрицу преобразования Денавитта-Хартенберга"""
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        T = np.array([
            [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
            [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])
        return T
    
    def forward_kinematic(self, joint_angles):
        """Прямая кинематика"""
        T = np.eye(4)
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            T_i = self.dh_matrix(a, alpha, d, theta)
            T = T @ T_i
        return T
    
    def _pose_error(self, current_pose, target_pose):
        """Вычисление ошибки между текущей и целевой позой"""
        # Ошибка позиции
        pos_error = np.linalg.norm(current_pose[:3, 3] - target_pose[:3, 3])
        
        # Ошибка ориентации через угол поворота
        R_current = current_pose[:3, :3]
        R_target = target_pose[:3, :3]
        R_diff = R_current.T @ R_target
        angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
        rot_error = np.abs(angle)
        
        return 0.9 * pos_error + 0.1 * rot_error
    
    def _objective_function(self, joint_angles, target_pose):
        """Целевая функция для оптимизации"""
        current_pose = self.forward_kinematic(joint_angles)
        return self._pose_error(current_pose, target_pose)
    
    def inverse_kinematic_trust_constr(self, target_pose, initial_guess=None, max_iterations=200):
        """
        Обратная кинематика с использованием Trust Region Constrained Algorithm
        """
        if initial_guess is None:
            initial_guess = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Функция для оптимизации
        def objective(joint_angles):
            return self._objective_function(joint_angles, target_pose)
        
        # Ограничения в виде bounds
        bounds = self.joint_limits
        
        try:
            result = minimize(
                objective,
                initial_guess,
                method='trust-constr',
                bounds=bounds,
                options={
                    'maxiter': max_iterations,
                    'verbose': 0,
                    'gtol': 1e-6,
                    'xtol': 1e-6
                }
            )
            
            final_pose = self.forward_kinematic(result.x)
            final_error = self._pose_error(final_pose, target_pose)
            success = final_error < 1e-4 and result.success
            
            return result.x, success, final_error
            
        except Exception as e:
            warnings.warn(f"Trust-constr failed: {e}")
            # Fallback to L-BFGS-B
            return self.inverse_kinematic_lbfgsb(target_pose, initial_guess, max_iterations)
    
    def inverse_kinematic_lbfgsb(self, target_pose, initial_guess=None, max_iterations=200):
        """L-BFGS-B с улучшенными настройками"""
        if initial_guess is None:
            initial_guess = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        def objective(joint_angles):
            return self._objective_function(joint_angles, target_pose)
        
        result = minimize(
            objective,
            initial_guess,
            method='L-BFGS-B',
            bounds=self.joint_limits,
            options={
                'maxiter': max_iterations,
                'ftol': 1e-8,
                'gtol': 1e-6,
                'eps': 1e-8
            }
        )
        
        final_pose = self.forward_kinematic(result.x)
        final_error = self._pose_error(final_pose, target_pose)
        success = final_error < 1e-4 and result.success
        
        return result.x, success, final_error