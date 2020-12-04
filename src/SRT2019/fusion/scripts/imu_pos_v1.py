#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# 实现了一个订阅器的功能，订阅imu的位置信息，并持续处理接受到的信息

from __future__ import absolute_import, division, print_function
import os
import sys
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Imu
from visualization_msgs.msg import *

if __name__ == "__main__":
    # 定义了一个名为image_process的节点，并且规定以后在定义相同名字的node时，按照序号进行排列(anonymous=True)
    rospy.init_node('image_process', anonymous=True)
    # 获得当前的位置信息
    pos = Pos();
    # 订阅了一个'/mynteye/imu/data_raw'的主题，接收消息的类型为Imu，每次接收到消息会呼叫callback函数
    # 使用queue_size=1的目的是使得当前函数处理的信息为最新接收到的，防止队列过长导致的延迟现象
    # buff_size为缓冲区的大小，应设置成一个大于一个消息大小的数值
    rospy.Subscriber('/mynteye/imu/data_raw', Imu, pos.callback, queue_size=1, buff_size=52428800)
    # spin()使得程序在被手动停止前持续运行
    rospy.spin()
