#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#应用lidar2img中的函数进行坐标的二维映射

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

transf_x = 0.80
transf_y = -0.06
transf_z = -0.55
Threshold = 150
Dist_move = 0.6
L = 1.0#wheelbase
Ld = 2.0#the foresee distance
Stop_coef = 12

#应用三维坐标向二维坐标的映射求得对应二维坐标，并输出值
if __name__ == "__main__":
    #x,y,z为位置的三维坐标，返回值u,v为所求的映射二维坐标
    x = 1.35214995146
    y = -1.14549875259
    z = -0.639716598392
    u, v = lidar2img.lidar2img(x, y, z)
    print u,v
