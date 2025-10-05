import cv2
import numpy as np
from ultralytics import YOLO
from typing import List, Dict, Tuple, Optional


class ObjectDetector:
    
    def __init__(self, model_path: str = '../../models/yolo11n.pt', confidence_threshold: float = 0.5):

        print(f"Загрузка модели YOLO: {model_path}...")
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        
        # Хранилище для последних детекций
        self.last_detections = []
        self.last_processed_image = None
        
        # Цвета для отрисовки bbox (BGR формат)
        self.colors = {
            # Ваши оригинальные цвета (сохранены)
            'orange': (0, 255, 0),           # зеленый
            'apple': (255, 0, 0),           # синий
            'cup': (0, 165, 255),            # оранжевый
            'wine glass': (147, 20, 255),    # розовый
            'default': (255, 255, 255)
        }
        
        print("Модель успешно загружена!")

    def get_classes(self):
        return ['orange', 'apple', 'cup', 'wine glass', 'default']
    
    def detect(self, image: np.ndarray) -> Tuple[np.ndarray, List[Dict]]:
        # Копируем изображение для отрисовки
        output_image = image.copy()
        
        # Запускаем детекцию
        results = self.model(image, conf=self.confidence_threshold, verbose=False)
        
        # Список для хранения найденных объектов
        detected_objects = []
        
        # Обрабатываем результаты
        for result in results:
            boxes = result.boxes
            
            for box in boxes:
                # Получаем координаты bbox
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Получаем класс, имя класса и уверенность
                cls = int(box.cls[0])
                class_name = result.names[cls]
                confidence = float(box.conf[0])
                
                # Вычисляем центр и размеры bbox
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                width = x2 - x1
                height = y2 - y1
                
                # Добавляем информацию об объекте в список
                obj_info = {
                    'class_id': cls,
                    'class_name': class_name,
                    'confidence': confidence,
                    'bbox': {
                        'x1': x1,
                        'y1': y1,
                        'x2': x2,
                        'y2': y2,
                        'center_x': center_x,
                        'center_y': center_y,
                        'width': width,
                        'height': height
                    }
                }
                detected_objects.append(obj_info)
                
                # Отрисовываем bbox на изображении
                output_image = self._draw_detection(
                    output_image, 
                    class_name, 
                    confidence, 
                    x1, y1, x2, y2,
                    center_x, center_y
                )
        
        # Сохраняем последние детекции для доступа через геттеры
        self.last_detections = detected_objects
        self.last_processed_image = output_image
        
        # Добавляем общую информацию о количестве найденных объектов
        self._draw_info_header(output_image, len(detected_objects))
        
        return output_image, detected_objects
    
    def _draw_detection(
        self, 
        image: np.ndarray, 
        class_name: str, 
        confidence: float, 
        x1: int, y1: int, x2: int, y2: int,
        center_x: int, center_y: int
    ) -> np.ndarray:
        # Выбираем цвет для bbox
        color = self.colors.get(class_name, self.colors['default'])
        
        # Отрисовываем прямоугольник bbox
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        
        # Отрисовываем центр объекта
        cv2.circle(image, (center_x, center_y), 5, color, -1)
        cv2.circle(image, (center_x, center_y), 8, (255, 255, 255), 2)
        
        # Подготавливаем текст с названием и уверенностью
        label = f"{class_name}: {confidence:.2%}"
        
        # Вычисляем размер текста для фона
        (text_width, text_height), baseline = cv2.getTextSize(
            label, 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.6, 
            2
        )
        
        # Отрисовываем фон для текста
        cv2.rectangle(
            image, 
            (x1, y1 - text_height - baseline - 5), 
            (x1 + text_width + 5, y1), 
            color, 
            -1
        )
        
        # Отрисовываем текст
        cv2.putText(
            image, 
            label, 
            (x1 + 2, y1 - 5), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.6, 
            (255, 255, 255), 
            2
        )
        
        # Добавляем координаты центра
        center_label = f"({center_x}, {center_y})"
        cv2.putText(
            image,
            center_label,
            (center_x - 40, center_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )
        
        return image
    
    def _draw_info_header(self, image: np.ndarray, objects_count: int) -> None:
        info_text = f"Objects found: {objects_count}"
        
        # Фон для текста
        cv2.rectangle(image, (5, 5), (350, 45), (0, 0, 0), -1)
        cv2.rectangle(image, (5, 5), (350, 45), (0, 255, 0), 2)
        
        # Текст
        cv2.putText(
            image, 
            info_text, 
            (15, 32), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.8, 
            (0, 255, 0), 
            2
        )
    
    def detect_from_file(self, image_path: str) -> Tuple[Optional[np.ndarray], List[Dict]]:
        # Загружаем изображение
        image = cv2.imread(image_path)
        
        if image is None:
            print(f"Ошибка: не удалось загрузить изображение {image_path}")
            return None, []
        
        return self.detect(image)
    
    def set_confidence_threshold(self, threshold: float) -> None:
        if 0.0 <= threshold <= 1.0:
            self.confidence_threshold = threshold
            print(f"Порог уверенности установлен: {threshold}")
        else:
            print("Ошибка: порог должен быть в диапазоне 0.0 - 1.0")
    
    def print_detection_results(self, detected_objects: List[Dict]) -> None:
        print("\n" + "="*60)
        print(f"РЕЗУЛЬТАТЫ ДЕТЕКЦИИ: Найдено объектов - {len(detected_objects)}")
        print("="*60)
        
        if not detected_objects:
            print("Объекты не обнаружены")
            return
        
        for i, obj in enumerate(detected_objects, 1):
            print(f"\nОбъект #{i}:")
            print(f"  Класс: {obj['class_name']} (ID: {obj['class_id']})")
            print(f"  Уверенность: {obj['confidence']:.2%}")
            print(f"  Координаты bbox:")
            print(f"    Верхний левый угол: ({obj['bbox']['x1']}, {obj['bbox']['y1']})")
            print(f"    Нижний правый угол: ({obj['bbox']['x2']}, {obj['bbox']['y2']})")
            print(f"    Центр: ({obj['bbox']['center_x']}, {obj['bbox']['center_y']})")
            print(f"    Размер: {obj['bbox']['width']}x{obj['bbox']['height']} px")
        
        print("="*60 + "\n")
    
    # НОВЫЕ ГЕТТЕРЫ ДЛЯ ПОЛУЧЕНИЯ КООРДИНАТ ЦЕНТРОВ
    
    def get_last_detections(self) -> List[Dict]:
        """Возвращает список всех последних обнаруженных объектов"""
        return self.last_detections.copy()
    
    def get_object_center(self, index: int = 0) -> Optional[Tuple[int, int]]:
        """
        Возвращает центр (x, y) объекта по индексу
        
        Args:
            index: индекс объекта в списке детекций (по умолчанию 0 - первый объект)
            
        Returns:
            Tuple[int, int] или None если объект не найден
        """
        if not self.last_detections or index >= len(self.last_detections):
            return None
        
        obj = self.last_detections[index]
        return (obj['bbox']['center_x'], obj['bbox']['center_y'])
    
    def get_object_centers_by_class(self, class_name: str) -> List[Tuple[int, int]]:
        """
        Возвращает список центров всех объектов указанного класса
        
        Args:
            class_name: имя класса для фильтрации
            
        Returns:
            List[Tuple[int, int]] - список координат центров
        """
        centers = []
        for obj in self.last_detections:
            if obj['class_name'].lower() == class_name.lower():
                centers.append((obj['bbox']['center_x'], obj['bbox']['center_y']))
        return centers
    
    def get_highest_confidence_center(self) -> Optional[Tuple[int, int]]:
        """
        Возвращает центр объекта с наибольшей уверенностью
        
        Returns:
            Tuple[int, int] или None если объектов нет
        """
        if not self.last_detections:
            return None
        
        # Находим объект с максимальной уверенностью
        best_obj = max(self.last_detections, key=lambda x: x['confidence'])
        return (best_obj['bbox']['center_x'], best_obj['bbox']['center_y'])
    
    def get_all_centers(self) -> List[Tuple[int, int]]:
        """
        Возвращает центры всех обнаруженных объектов
        
        Returns:
            List[Tuple[int, int]] - список всех центров
        """
        return [(obj['bbox']['center_x'], obj['bbox']['center_y']) 
                for obj in self.last_detections]
    
    def get_object_info(self, index: int = 0) -> Optional[Dict]:
        """
        Возвращает полную информацию об объекте по индексу
        
        Args:
            index: индекс объекта
            
        Returns:
            Dict или None если объект не найден
        """
        if not self.last_detections or index >= len(self.last_detections):
            return None
        return self.last_detections[index].copy()
    
    def get_objects_count(self) -> int:
        """Возвращает количество обнаруженных объектов"""
        return len(self.last_detections)
    
    def has_detections(self) -> bool:
        """Проверяет, есть ли обнаруженные объекты"""
        return len(self.last_detections) > 0
    
    def get_last_processed_image(self) -> Optional[np.ndarray]:
        """Возвращает последнее обработанное изображение с детекциями"""
        return self.last_processed_image.copy() if self.last_processed_image is not None else None