import cv2
import numpy as np
from ultralytics import YOLO
from typing import List, Dict, Tuple, Optional


class ObjectDetector:
    
    def __init__(self, model_path: str = '../../models/yolo11n.pt', confidence_threshold: float = 0.5):

        print(f"Загрузка модели YOLO: {model_path}...")
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        
        # Цвета для отрисовки bbox (BGR формат)
        self.colors = {
            'orange': (0, 255, 0),        # зеленый
            'sports ball': (255, 0, 0), # синий
            'can': (0, 165, 255),       # оранжевый
            'wine glass': (147, 20, 255), # розовый
            'default': (255, 255, 255)  # белый
        }
        
        print("Модель успешно загружена!")
    
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
                    x1, y1, x2, y2
                )
        
        # Добавляем общую информацию о количестве найденных объектов
        self._draw_info_header(output_image, len(detected_objects))
        
        return output_image, detected_objects
    
    def _draw_detection(
        self, 
        image: np.ndarray, 
        class_name: str, 
        confidence: float, 
        x1: int, y1: int, x2: int, y2: int
    ) -> np.ndarray:

        # Выбираем цвет для bbox
        color = self.colors.get(class_name, self.colors['default'])
        
        # Отрисовываем прямоугольник bbox
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        
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
        
        return image
    
    def _draw_info_header(self, image: np.ndarray, objects_count: int) -> None:

        info_text = f"Найдено объектов: {objects_count}"
        
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


# def example_with_image_file():
    
#     # Создаем экземпляр детектора
#     detector = ObjectDetector(model_path='../../models/yolo11n.pt', confidence_threshold=0.3)
    
#     # Путь к изображению
#     image_path = 'orange_2.png'
    
#     # Детектируем объекты
#     output_image, detected_objects = detector.detect_from_file(image_path)
    
#     if output_image is not None:
#         # Выводим результаты в консоль
#         detector.print_detection_results(detected_objects)
        
#         # Показываем результат
#         cv2.imshow('Detection Result', output_image)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
        
#         # Сохраняем результат
#         cv2.imwrite('result.png', output_image)
#         print("Результат сохранен в result.png")

# if __name__ == "__main__":
#     # Выберите нужный пример:
    
#     # Пример 1: Детекция на одном изображении
#     example_with_image_file()
