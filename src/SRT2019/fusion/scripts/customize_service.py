# Copyright 2019 ModelArts Authors from Huawei Cloud. All Rights Reserved.
# https://www.huaweicloud.com/product/modelarts.html
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
import ast
import io
import os
import time

import numpy as np
import tensorflow as tf

import h5py
from model_service.tfserving_model_service import TfServingBaseService
from PIL import Image

EPS = np.finfo(float).eps


class object_detection_service(TfServingBaseService):

  def _preprocess(self, data):
    preprocessed_data = {}
    for k, v in data.items():
      for file_name, file_content in v.items():
        image = Image.open(file_content)
        image = image.convert('RGB')
        image = np.asarray(image, dtype=np.float32)
        image = image[np.newaxis, :, :, :]
        preprocessed_data[k] = image
    return preprocessed_data

  def _postprocess(self, data):
    h5f = h5py.File(os.path.join(self.model_path, 'index'), 'r')
    labels_list = h5f['labels_list'][:]
    h5f.close()
    num_boxes = len(data['detection_classes'])
    classes = []
    boxes = []
    scores = []
    prob_threshold = 0.3
    result_return = dict()
    for i in range(num_boxes):
      if data['detection_scores'][i] > prob_threshold:
        class_id = data['detection_classes'][i] - 1
        classes.append(labels_list[int(class_id)])
        boxes.append(data['detection_boxes'][i])
        scores.append(data['detection_scores'][i])
    ##########add NMS#######################################
    nms_iou_threshold = 0.3
    bounding_boxes = boxes
    confidence_score = scores
    # Bounding boxes
    boxes = np.array(bounding_boxes)
    picked_boxes = []
    picked_score = []
    picked_classes = []
    if len(boxes) != 0:
      # coordinates of bounding boxes
      start_x = boxes[:, 0]
      start_y = boxes[:, 1]
      end_x = boxes[:, 2]
      end_y = boxes[:, 3]
      # Confidence scores of bounding boxes
      score = np.array(confidence_score)
      # Picked bounding boxes
      # Compute areas of bounding boxes
      areas = (end_x - start_x + 1) * (end_y - start_y + 1)
      # Sort by confidence score of bounding boxes
      order = np.argsort(score)
      # Iterate bounding boxes
      while order.size > 0:
        # The index of largest confidence score
        index = order[-1]
        # Pick the bounding box with largest confidence score
        picked_boxes.append(bounding_boxes[index])
        picked_score.append(confidence_score[index])
        picked_classes.append(classes[index])
        # Compute ordinates of intersection-over-union(IOU)
        x1 = np.maximum(start_x[index], start_x[order[:-1]])
        x2 = np.minimum(end_x[index], end_x[order[:-1]])
        y1 = np.maximum(start_y[index], start_y[order[:-1]])
        y2 = np.minimum(end_y[index], end_y[order[:-1]])
        # Compute areas of intersection-over-union
        w = np.maximum(0.0, x2 - x1 + 1)
        h = np.maximum(0.0, y2 - y1 + 1)
        intersection = w * h
        # Compute the ratio between intersection and union
        ratio = intersection / (areas[index] + areas[order[:-1]] - intersection + EPS)
        left = np.where(ratio < nms_iou_threshold)
        order = order[left]

    result_return['detection_classes'] = picked_classes
    result_return['detection_boxes'] = picked_boxes
    result_return['detection_scores'] = picked_score
    return result_return
