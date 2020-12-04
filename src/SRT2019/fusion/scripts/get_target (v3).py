#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
import sympy
import math
#该文件中的函数通过给定一列探测到的cone的位置(con_pos), 处理并给出行进目标点的坐标，求解目标点时的参数系数与v2不同
#Note:     the positions of the cones and the target are under the autonomous car's local coordinate system
FORWARD_DISTANCE = 3.0
TRANSLATION_DISTANCE = 1.5
CRITICAL_VALUE = 3.5
ADD_VALUE = 1.2

#函数名: coordination_transformation1
#功能: 对输入的点进行坐标变换
#输入： point:二维坐标
#输出: 二维坐标
def coordination_transformation1(point):
    tmp = point[0]
    point[0] = -point[1]
    point[1] = tmp
    return point

#函数名: coordination_transformation2
#功能: 对输入的点进行坐标变换
#输入： point:二维坐标
#输出: 二维坐标
def coordination_transformation2(point):
    tmp = point[0]
    point[0] = point[1]
    point[1] = -tmp
    return point

#函数名: medline
#功能: 给定两点坐标，给出通过它们中点的直线的系数(似乎不是垂直平分线. 不知道为什么相比v1v2又变回去了)
#输入: x1,y1,x2,y2: 横纵坐标
#输出:Ax+By+C=0的ABC三个系数
def medLine(x1,y1,x2,y2): # Ax+By+C=0
    A = (y1+y2)/2
    B = -(x1+x2)/2
    C = 0
    return A,B,C

#函数名: two_points_getaline
#功能: 给定两点坐标，给出通过它们的直线的系数(似乎x2==x1情况输出不对???)
#输入: x1,y1,x2,y2: 横纵坐标
#输出:Ax+By+C=0的ABC三个系数
def two_points_getaline(x1,y1,x2,y2):
    if x2==x1:
        A = 0
        B = 1
        C = x1
    else:
        k = (y2-y1)/(x2-x1)
        b = y1 - k*x1
        A = k
        B = -1
        C = b
    return A,B,C

#函数名:to_float
#功能: 将形如{x:1.5,y:2.0}的字符串,中的数字(float)提取出来并返回
#输入: target: 形如{x:1.5,y:2.0}的字符串
#输出: 两个浮点数
def to_float(target):
    target_str = str(target)
    idx = target_str.find('x:')
    idy = target_str.find('y:')
    idx_stop = target_str.find(',')
    idy_stop = target_str.find('}')
    subx = target_str[idx+2:idx_stop]
    suby = target_str[idy+2:idy_stop]
    x = float(subx)
    y = float(suby) 
    #print x
    #print y
    return [x,y]

#函数名: get_the_point
#功能: 给定一个point的list, 返回第二个点
#输入:points: 点的list
#输出: 点坐标(list)
def get_the_point(points): #points is a list
    point1 = list(points[0])
    point2 = list(points[1])
    print point1[0]
    print point2[0]
    if point1[0] >= 0:
        return point1
    else :
        return point2

#函数名:get_target
#功能: 给出一列cone_pos对象, 给出车的行进目标点的坐标(通过求一条直线与一个圆的交点)
#输入: cone_whole: con_pos对象的list
#输出: 一个二维坐标，表示行进目标点
def get_target(cone_whole):
    '''
    #Requires: the input cone_whole is an array of cone_pos objects,
    #          which contains the x,y and color of a detected cone
    #Effects:  get the navigation target for the autonomous car based
    #          on given detected cones. The output target_points is 
    #          the x,y coordinate of the target.only
    #Note:     the positions of the cones and the target are under
    #          the autonomous car's local coordinate system
    '''
    lane_left = [] #左侧的cone_pos集合(红色)
    lane_right = [] #右侧的cone_pos集合(蓝色)
    for cone in cone_whole:
        cone_list = [cone.x,cone.y]
        coordination_transformation1(cone_list)
	if cone.color == 'r':
            lane_left.append(cone)
        elif cone.color == 'b':
            lane_right.append(cone)

    if len(lane_left) >= len(lane_right):
        major_lane = lane_left
        secondary_lane = lane_right
        major_lane_color = 'r'
    else:
        major_lane = lane_right
        secondary_lane = lane_left
        major_lane_color = 'b'

    # 求解方程获得目标点坐标
    if len(major_lane) >= 2:
        A,B,C = two_points_getaline(major_lane[0].x + ADD_VALUE, major_lane[0].y , major_lane[1].x + ADD_VALUE, major_lane[1].y)
        k = TRANSLATION_DISTANCE/math.sqrt(A**2+B**2)
        if major_lane_color == 'r':
            C = C-A*k*A-B*k*B
        else:
            C = C+A*k*A+B*k*B  
        x,y = sympy.symbols("x y")
        target_point_solve = sympy.solve([A*x+B*y+C, x**2+y**2-FORWARD_DISTANCE**2],[x,y])
        target_point1 = get_the_point(target_point_solve)
        return target_point1
        #target_point = list(to_float(target_point1))
        #return coordination_transformation2(target_point)
    elif len(major_lane) == 1:
        if len(secondary_lane) == 1:
            #if math.sqrt((major_lane[0].x-secondary_lane[0].x)**2 + (major_lane[0].y-secondary_lane[0].y)**2) > CRITICAL_VALUE:
            #    return -1,0 
            A,B,C = medLine(major_lane[0].x + ADD_VALUE, major_lane[0].y , secondary_lane[0].x + ADD_VALUE, secondary_lane[0].y)
            C = 0
            x,y = sympy.symbols("x y")
            target_point_solve = sympy.solve([A*x+B*y+C, x**2+y**2-FORWARD_DISTANCE**2],[x,y])
            target_point1 = get_the_point(target_point_solve)
            return target_point1
            #target_point = list(to_float(target_point1))
            #return coordination_transformation2(target_point)
        else: #only one point
            return [-1, 0]
    else:
        return  [0, 0]

