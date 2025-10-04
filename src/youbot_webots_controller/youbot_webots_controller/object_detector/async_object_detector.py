
import cv2
import numpy as np
import time
from threading import Thread, Lock
from enum import Enum
from typing import List, Dict, Tuple, Optional
from youbot_webots_controller.object_detector.object_detector import ObjectDetector


class DetectorState(Enum):
    """Состояния детектора"""
    IDLE = "idle"                    # Готов к работе, ничего не обрабатывает
    PROCESSING = "processing"        # Идет обработка изображения
    COMPLETED = "completed"          # Обработка завершена, результат готов


class AsyncObjectDetector:

    
    def __init__(self, model_path: str = '../../models/yolo11n.pt', 
                 confidence_threshold: float = 0.5):
        
        self.detector = ObjectDetector(model_path, confidence_threshold)
        
        # Состояние детектора
        self._state = DetectorState.IDLE
        self._lock = Lock()
        
        # Данные для обработки и результаты
        self._input_image = None
        self._output_image = None
        self._detected_objects = None
        
        # Поток для обработки
        self._processing_thread = None
        
        print("Object detector has been initialized")
    
    def is_idle(self) -> bool:

        with self._lock:
            return self._state == DetectorState.IDLE
    
    def is_processing(self) -> bool:

        with self._lock:
            return self._state == DetectorState.PROCESSING
    
    def is_completed(self) -> bool:

        with self._lock:
            return self._state == DetectorState.COMPLETED
    
    def get_state(self) -> DetectorState:

        with self._lock:
            return self._state
    
    def process_image(self, image: np.ndarray) -> bool:

        with self._lock:
            # Проверяем, свободен ли детектор
            if self._state != DetectorState.IDLE:
                print("Detector busy! Try again later")
                return False
            
            # Сохраняем изображение и меняем состояние
            self._input_image = image.copy()
            self._state = DetectorState.PROCESSING
            self._output_image = None
            self._detected_objects = None
        
        # Запускаем обработку в отдельном потоке
        self._processing_thread = Thread(target=self._process_image, daemon=True)
        self._processing_thread.start()
        
        return True
    
    def get_result(self) -> tuple:

        with self._lock:
            if self._state == DetectorState.COMPLETED:
                # Возвращаем результат и переводим детектор в состояние IDLE
                output_image = self._output_image
                detected_objects = self._detected_objects
                
                # Очищаем результаты и переводим в режим ожидания
                self._state = DetectorState.IDLE
                self._input_image = None
                self._output_image = None
                self._detected_objects = None
                
                return output_image, detected_objects
            else:
                return None, None
    
    def _process_image(self):

        try:
            # Получаем изображение для обработки
            with self._lock:
                image = self._input_image.copy()
            
            # Выполняем детекцию (это может занять время)
            output_image, detected_objects = self.detector.detect(image)
            
            # Сохраняем результаты
            with self._lock:
                self._output_image = output_image
                self._detected_objects = detected_objects
                self._state = DetectorState.COMPLETED
                
            #print(f"Processing finished. Objects find: {len(detected_objects)}")
            
        except Exception as e:
            print(f"Image processing error: {e}")
            with self._lock:
                self._state = DetectorState.IDLE
                self._output_image = None
                self._detected_objects = None
    
    def wait_for_completion(self, timeout: float = None) -> bool:

        start_time = time.time()
        
        while True:
            if self.is_completed():
                return True
            
            if timeout and (time.time() - start_time) > timeout:
                return False
            
            time.sleep(0.01)  # Небольшая задержка для снижения нагрузки на CPU

    def get_highest_confidence_center(self) -> Optional[Tuple[int, int]]:
        return self.detector.get_highest_confidence_center()

    
    def reset(self):

        with self._lock:
            self._state = DetectorState.IDLE
            self._input_image = None
            self._output_image = None
            self._detected_objects = None
        print("Детектор сброшен в состояние IDLE")