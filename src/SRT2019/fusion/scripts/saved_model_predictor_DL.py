#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from tensorflow.contrib.predictor.saved_model_predictor import _get_signature_def
from tensorflow.contrib.predictor.saved_model_predictor import _check_signature_arguments
from tensorflow.contrib.predictor.predictor import Predictor
import tensorflow as tf

#本文件提供了加载训练好的模型并使用tensorflow利用该模型进行预测的方法, 在detection.py中, 调用了该方法以对图像中的物品进行识别

class SavedModelPredictor(Predictor):
  """A `Predictor` constructed from a `SavedModel`."""

  def __init__(self,
               export_dir, #存储SavedModel协议缓冲区和要加载的变量的目录
               signature_def_key=None,
               signature_def=None,
               input_names=None,
               output_names=None,
               tags=None, #用于恢复元图的标, 区别不同的模型
               graph=None):
    _check_signature_arguments(
        signature_def_key, signature_def, input_names, output_names)
    tags = tags or "serve" #serve从预定义的SavedModel取得常量标签
    self._graph = graph or tf.get_default_graph()

    gpu_options = tf.GPUOptions(allow_growth=True)
    config = tf.ConfigProto(gpu_options=gpu_options)

    with self._graph.as_default():   #使用默认的tensorflow运算流程图
      self._session = tf.Session(config=config) #tensorflow会话
      tf.saved_model.loader.load(self._session, tags.split(','), export_dir) #向此会话加载预训练模型

    if input_names is None:
      if signature_def is None:
        signature_def = _get_signature_def(signature_def_key, export_dir, tags)
      input_names = {k: v.name for k, v in signature_def.inputs.items()}
      output_names = {k: v.name for k, v in signature_def.outputs.items()}

    self._feed_tensors = {k: self._graph.get_tensor_by_name(v)
                          for k, v in input_names.items()}
    self._fetch_tensors = {k: self._graph.get_tensor_by_name(v)
                           for k, v in output_names.items()}
