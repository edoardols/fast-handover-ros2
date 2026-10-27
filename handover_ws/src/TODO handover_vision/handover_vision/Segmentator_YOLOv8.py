import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class SegmentatorYOLOv8:
    def __init__(self, model_path="yolov8n-seg.pt", conf_threshold=0.5):
        if YOLO is None:
            raise RuntimeError("Ultralytics non installato. Installa con: pip install ultralytics")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold

    def segment(self, image: np.ndarray):
        results = self.model(image, conf=self.conf_threshold, verbose=False)
        masks = []
        boxes = []
        labels = []

        if len(results) > 0 and results[0].masks is not None:
            for mask, box, cls in zip(results[0].masks.data, results[0].boxes.xyxy, results[0].boxes.cls):
                mask_np = mask.cpu().numpy().astype(np.uint8) * 255
                mask_resized = cv2.resize(mask_np, (image.shape[1], image.shape[2] if len(image.shape)>2 else image.shape[0]))
                masks.append(mask_resized)
                boxes.append(box.cpu().numpy())
                labels.append(int(cls.cpu().numpy()))

        return masks, boxes, labels