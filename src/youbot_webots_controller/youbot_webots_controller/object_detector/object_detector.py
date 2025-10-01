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
            # Ваши оригинальные цвета (сохранены)
            'orange': (0, 255, 0),           # зеленый
            'bottle': (255, 0, 0),           # синий
            'can': (0, 165, 255),            # оранжевый
            'wine glass': (147, 20, 255),    # розовый
            
            # Новые цвета для остальных классов
            'person': (255, 255, 0),         # голубоватый
            'bicycle': (255, 0, 255),        # пурпурный
            'car': (0, 255, 255),            # желтый
            'motorcycle': (128, 0, 128),     # фиолетовый
            'airplane': (255, 165, 0),       # оранжевый
            'bus': (0, 128, 128),            # темно-бирюзовый
            'train': (128, 128, 0),          # оливковый
            'truck': (75, 0, 130),           # индиго
            'boat': (240, 128, 128),         # светло-коралловый
            'traffic light': (0, 255, 127),  # весенне-зеленый
            'fire hydrant': (220, 20, 60),   # малиновый
            'stop sign': (178, 34, 34),      # огненный кирпич
            'parking meter': (165, 42, 42),  # коричневый
            'bench': (210, 105, 30),         # шоколадный
            'bird': (64, 224, 208),          # бирюзовый
            'cat': (255, 192, 203),          # розовый
            'dog': (139, 69, 19),            # седло-коричневый
            'horse': (160, 82, 45),          # сиена
            'sheep': (255, 250, 250),        # снежный
            'cow': (105, 105, 105),          # темно-серый
            'elephant': (169, 169, 169),     # темно-серый
            'bear': (84, 84, 84),            # серый
            'zebra': (245, 245, 245),        # белый дым
            'giraffe': (255, 215, 0),        # золотой
            'backpack': (0, 0, 139),         # темно-синий
            'umbrella': (72, 61, 139),       # темно-синевато-серый
            'handbag': (106, 90, 205),       # сланец синий
            'tie': (123, 104, 238),          # сланец синий 2
            'suitcase': (65, 105, 225),      # королевский синий
            'frisbee': (30, 144, 255),       # голубой
            'skis': (135, 206, 250),         # светло-голубой
            'snowboard': (70, 130, 180),     # стальной синий
            'sports ball': (100, 149, 237),  # васильковый
            'kite': (176, 196, 222),         # светло-стальной синий
            'baseball bat': (255, 99, 71),   # томатный
            'baseball glove': (255, 127, 80),# коралловый
            'skateboard': (205, 92, 92),     # индийский красный
            'surfboard': (250, 128, 114),    # лососевый
            'tennis racket': (233, 150, 122),# темный лосось
            'cup': (255, 160, 122),          # светло-лососевый
            'fork': (255, 69, 0),            # красно-оранжевый
            'knife': (255, 140, 0),          # темно-оранжевый
            'spoon': (255, 165, 0),          # оранжевый
            'bowl': (255, 218, 185),         # персиковый
            'banana': (255, 255, 0),         # желтый
            'apple': (255, 0, 0),            # красный
            'sandwich': (210, 180, 140),     # танг
            'broccoli': (0, 128, 0),         # зеленый
            'carrot': (255, 140, 0),         # темно-оранжевый
            'hot dog': (165, 42, 42),        # коричневый
            'pizza': (255, 69, 0),           # красно-оранжевый
            'donut': (210, 105, 30),         # шоколадный
            'cake': (255, 192, 203),         # розовый
            'chair': (139, 0, 0),            # темно-красный
            'couch': (160, 82, 45),          # сиена
            'potted plant': (34, 139, 34),   # лесной зеленый
            'bed': (128, 0, 0),              # темно-бордовый
            'dining table': (184, 134, 11),  # темно-золотой
            'toilet': (192, 192, 192),       # серебряный
            'tv': (47, 79, 79),              # темно-серый синеватый
            'laptop': (105, 105, 105),       # темно-серый
            'mouse': (128, 128, 128),        # серый
            'remote': (169, 169, 169),       # темно-серый
            'keyboard': (112, 128, 144),     # сланец серый
            'cell phone': (119, 136, 153),   # светло-сланцевый серый
            'microwave': (47, 79, 79),       # темно-серый синеватый
            'oven': (133, 133, 133),         # серый
            'toaster': (141, 141, 141),      # серый
            'sink': (189, 183, 107),         # темный хаки
            'refrigerator': (85, 107, 47),   # темный оливково-зеленый
            'book': (139, 69, 19),           # седло-коричневый
            'clock': (72, 61, 139),          # темно-синевато-серый
            'vase': (219, 112, 147),         # бледно-фиолетовый красный
            'scissors': (176, 48, 96),       # тусклый фиолетовый
            'teddy bear': (255, 20, 147),    # темно-розовый
            'hair drier': (199, 21, 133),    # средний фиолетовый красный
            'toothbrush': (219, 112, 147),   # бледно-фиолетовый красный
            'default': (255, 255, 255)       # белый
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
