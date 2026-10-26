import os
import torch
from ultralytics import YOLO
import numpy as np
import cv2

class Segmentator:
    def __init__(self, conf_thres, weights):
        self.model = YOLO(weights)  # load an official model
        self.line_thickness = 0.1
        self.conf_thres = conf_thres


    # accepts a BGR image
    def predict(self, im: np.ndarray):

        masks = None
        im0 = im.copy()
        results = self.model(im, conf=self.conf_thres, stream=False, verbose=False, device=0)

        classes = list()

        # Process predictions
        for result in results:
            boxes = result.boxes  # Boxes object for bbox outputs
            if result.masks is not None:
                masks = result.masks.masks  # Masks object for segmentation masks outputs
                if isinstance(masks, torch.Tensor):
                    masks = torch.as_tensor(masks, dtype=torch.uint8)
                    masks = masks.permute(1, 2, 0).contiguous()
                    masks = masks.cpu().numpy()

                masks = self.scale_image(masks.shape[:2], masks, im0.shape)
            im0 = result.plot()

            for i in range(len(boxes.cls.cpu().numpy())):
                classes.append(boxes.cls.cpu().numpy()[i])

        return im0, masks, np.array(classes)

    def scale_image(self, im1_shape, masks, im0_shape, ratio_pad=None):

        if ratio_pad is None: 
            gain = min(im1_shape[0] / im0_shape[0], im1_shape[1] / im0_shape[1])  
            pad = (im1_shape[1] - im0_shape[1] * gain) / 2, (im1_shape[0] - im0_shape[0] * gain) / 2  
        else:
            pad = ratio_pad[1]


        top, left = int(pad[1]), int(pad[0])  # y, x
        bottom, right = int(im1_shape[0] - pad[1]), int(im1_shape[1] - pad[0])

        if len(masks.shape) < 2:
            raise ValueError(f'"len of masks shape" should be 2 or 3, but got {len(masks.shape)}')
        masks = masks[top:bottom, left:right]
        masks = cv2.resize(masks, (im0_shape[1], im0_shape[0]))

        if len(masks.shape) == 2:
            masks = masks[:, :, None]
        return masks








