#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import lidar2img
import numpy as np
from numpy import linalg as la
from fusion.msg import lidar3_single
from fusion.msg import lidar3_whole
from fusion.msg import imagel_single
from fusion.msg import imagel_whole
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
from fusion.msg import Srt_Control
import get_target
import math
#该文件用于测试get_target中定义的函数是否能正确运行
if __name__ == "__main__":
    cone1 = cone_pos()
    cone2 = cone_pos()
    cone1.x = 2.9945142746
    cone1.y = -0.670192182064
    cone1.color = 'r'
    cone2.x = 3.68553667068
    cone2.y = -2.91794991493
    cone2.color = 'r'
    cones = [cone1,cone2]
    target = get_target.get_target(cones)
    print target
