#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#定义低通滤波类及计算欧氏距离的函数
#定义融合摄像头及雷达传回的原始信息，并以此操控车辆运行的类


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
transf_y = 0.0
transf_z = -0.55
Threshold = 60
Dist_move = 1.0
L = 1.0            #wheelbase
Ld = 3.0           #the foresee distance
Stop_coef = 12

# 定义了一个低通滤波类，使用一阶滞后滤波法进行实现
class LowpassFilter():
    #默认的构造函数，使用time_rate进行初始化
    def __init__(self, time_rate):
        self.__time_rate = time_rate  #在滞后表达式中使用，与滞后的程度有关
        self.__output = 0.0           #记录可以输出的数据
        self.__state = 0.0            #上一次的滤波结果
        self.__input = 0.0            #本次的采样值
    #从外部传入数据，设置Input值
    def giveInput(self, external_input):
        self.__input = external_input
    #将input传递至state及output中
    def initiallize(self):
        self.__state = self.__input
        self.__output = self.__input
    #低通滤波核心函数，使用算法为一阶滞后滤波法，本次滤波结果=（1-a）*本次采样值+a*上次滤波结果 (a为一个0-1之间的数)
    #此处time_rate相当于上式中的1-a
    def step(self):
        #若本次采样值与上次滤波结果差距过大，则忽略本次采样的结果
        if abs(self.__input - self.__state) > 2:
            self.__input = self.__state
        #使用上述公式计算出本次滤波结果，并记录到output和state中
        next_state = (1 - self.__time_rate) * self.__state + self.__time_rate * self.__input
        self.__output = next_state
        self.__state = next_state
    #获得输出值
    def getOutput(self):
        return self.__output

#定义了一个time_rate为0.25的低通滤波器
lp_filter = LowpassFilter(0.25)

#计算欧氏距离，使用numpy中求范数的函数进行等值计算
def Euc_dist(pos_1,pos_2):
    #函数原型np.linalg.norm(x, ord=None, axis=None, keepdims=False)
    #返回值为一个数，参数中x为矩阵，ord为范数类型，默认（同ord=2）为对平方和开根号，ord=1为绝对值之和，ord=np.inf为求元素绝对值最大值
    #axis：处理类型axis=1表示按行向量处理，求多个行向量的范数；axis=0表示按列向量处理，求多个列向量的范数；axis=None表示矩阵范数(默认None)
    #keepding：是否保持矩阵的二维特性；True表示保持矩阵的二维特性，False相反(默认为False)
    return la.norm(np.subtract(pos_1,pos_2))

#定义在二维图像上桶桩的类，其中u与v分别代表在两坐标轴上的投影
class cone_on_img():
    def __init__(self, u, v):
        self.u = u
        self.v = v

#识别桶桩过程中的信息融合
class cone_fusion():
    def __init__(self):
        #摄像机和雷达的原始图像数据
        self.lidar_raw = []
        self.image_raw = []
        #保存之前接收到的有效信息
        self.previous_points = []
        self.previous_target = []
        self.use_previous = 0       #是否在使用之前的信息，值为时间差
        self.stop = 0               #是否接收到停止信息，1表示停止
        #分别定义处理检测到的桶桩，目标点以及控制信息的订阅器
        self.fusion_pub = rospy.Publisher("/cone_detected", cone_pos_whole, queue_size = 10)
        self.target_pub = rospy.Publisher("/target_point",cone_pos,queue_size = 10)
        self.control_pub = rospy.Publisher("/srt_control",Srt_Control,queue_size = 10)

    #返回数组中最小元素
    def findMinI(self,l):
        length = len(l)
        mins = 0
        mini = l[0]

        for ii in range(1, length):
            if mini > l[ii]:
                mini = l[ii]
                mins = ii

        return mins 

    #匹配位置坐标，得到对应的颜色信息
    def match(self, u, v):
        #num为image_raw列表中最后一个元素（即最后获得图像信息）在列表中的下标
        num = len(self.image_raw) - 1
        #image_len为雷达图中搜索到点的个数
        image_len = len(self.image_raw[num].image_points)
        #用一矩形范围匹配在(u,v)附近点（即为桶桩），其中Threshold为范围的数值，并输出其颜色信息
        for ii in range(0, image_len):
            #print(u, v)
            #print(self.image_raw[num].image_points[ii].start_x, self.image_raw[num].image_points[ii].end_x, self.image_raw[num].image_points[ii].start_y, self.image_raw[num].image_points[ii].end_y)
            if (v > self.image_raw[num].image_points[ii].start_x - Threshold) and (v < self.image_raw[num].image_points[ii].end_x + Threshold) and (u > self.image_raw[num].image_points[ii].start_y - Threshold) and (u < self.image_raw[num].image_points[ii].end_y + Threshold):
                #print ("color == ")
                #print (self.image_raw[num].image_points[ii].color)
                return self.image_raw[num].image_points[ii].color
        return "unknown"
            


    def fusion(self, pose, dt): 
        num = len(self.image_raw) - 1
        #dist = []
        if num > -1:
            u, v = lidar2img.lidar2img(pose.x, pose.y, pose.z)
            jj = len(self.image_raw[num].image_points)
            if jj != 0:
                
                #for ii in range(0, jj):
                    #dist.append(self.Euc_dist(coneTmp, self.image_raw[num].image_points[jj - 1]))
                #ii = self.findMinI(dist)
                #cone为对cone_pos类的实例化，并对其进行对应信息的输入
                cone = cone_pos()
                cone.x = pose.x
                cone.y = pose.y
                #cone.color = self.image_raw[num].image_points[ii].color
                cone.color = self.match(u, v)
                #打印pose的各信息
                print ("pose x")
                print pose.x
                print ("pose y")
                print pose.y
                print ("pose z")
                print pose.z
                print ("pose color")
                print cone.color
                #将cone的信息添加到列表lidar_raw中
                self.lidar_raw.append(cone)

    #根据目标的位置信息计算驾驶的方式，包括角度，速度等
    def driving(self,target):
        sin_alpha = target[1] / Ld                                    #前进方向和目标连线的夹角
        steer_angle = math.degrees(math.atan2(2*L*sin_alpha/Ld,1) )   #转向角
        linear_velocity = 0.5                                         #轮上速度
        #若使用的为之前的数据，保持直行并将速度控制为0.3
        if self.use_previous > 0:
            steer_angle = 0.0
            linear_velocity = 0.3
        #若连续多次没有获得新数据或接收到停止命令，则摆正角度后停止
        if self.use_previous > Stop_coef or (target[0] == 0 and target[1] == 0) or self.stop == 1: 
            steer_angle = 0.0
            linear_velocity = 0.0
        #将信息传输至控制模块中
        control_param = Srt_Control()
        control_param.linear_velocity = linear_velocity
        control_param.steer_angle = steer_angle
        #在发布器中发布数据信息
        self.control_pub.publish(control_param)

    #对于雷达信息的callback
    def lidar_callback(self, msg):
        msgl = len(msg.lidar_points)        #激光雷达采集到点的数量
        if msgl != 0:
            #根据激光雷达和摄像机在空间上位置的差异，对采集到数据点进行对应平移
            for i in msg.lidar_points:
                i.x += transf_x
                i.y += transf_y
                i.z += transf_z
            #更新时间戳信息(即时间)
            time_stamp = msg.stamp
            jj = len(self.image_raw) - 1
            delta_t = 1.0
            #按照获取图像信息的新旧顺序，从最新的开始遍历，直到获得第一个在当前时间之前的图像信息
            while jj >= 0:
                if time_stamp > self.image_raw[jj].stamp:
                    delta_t = time_stamp - self.image_raw[jj].stamp
                    break
                jj = jj - 1
            # 为保证列表中至少存在一个元素，保留image_raw[jj]，删除下标为0到jj-1
            del self.image_raw[0:jj]

            #对于雷达中的点进行融合
            length_msg = len(msg.lidar_points)
            for ii in range(0, length_msg):
                self.fusion(msg.lidar_points[ii],delta_t)

            kk = len(self.lidar_raw)#originally: kk = len(self.lidar_raw) - 1
            length_prev = len(self.previous_points)
            for jj in range(0,kk):
                #该点为不在搜索范围内的点
                if self.lidar_raw[jj].color == "unknown":
                    for ll in range(0,length_prev):
                        #将该点的颜色信息更新为距离其较近的之前有效信息点，以排除偶然性
                        if  Euc_dist(np.array([[self.lidar_raw[jj].x],[self.lidar_raw[jj].y]]),np.array([[self.previous_points[ll].x],[self.previous_points[ll].y]])) < Dist_move:
                            #print "change current"
                            #print self.lidar_raw[jj]
                            #print "to"
                            #print self.previous_points[ll]
                            self.lidar_raw[jj].color = self.previous_points[ll].color
                            break
            self.previous_points = []           #清空前有效点列
            #将雷达图中点填充入前有效点序列中以供下一次使用，若有颜色为'y'则发出停车指令
            for ii in range(0,kk):
                self.previous_points.append(self.lidar_raw[ii])
                if self.lidar_raw[ii].color == 'y':
                    self.stop = 1
            cones_whole = cone_pos_whole(self.lidar_raw, time_stamp)
            self.fusion_pub.publish(cones_whole)
            print(cones_whole)

            target_point = cone_pos()                         #获得桶桩位置
            target = get_target.get_target(self.lidar_raw)    #目标点的位置
            #若有对应位置信息，则使用低通滤波后更新至前有效点列表中
            if target[0] != -1:
                '''
                #add discrete low pass filter here
                '''
                lp_filter.giveInput(target[1])
                lp_filter.step()
                target[1] = lp_filter.getOutput()
                self.use_previous = 0
                self.previous_target = target
            #若当前未检测到目标点，使用最近一次检测到的目标，并将时间差+1
            else:
                self.use_previous = self.use_previous + 1
                target = self.previous_target
            #更新使用目标点的信息
            target_point.x = target[0]
            target_point.y = target[1]
            target_point.color ="unknown"
            print "target"
            print target
            self.target_pub.publish(target_point)        #发布目标点的信息
            self.driving(target)                         #根据目标信息操控车辆运行的参数

            del self.lidar_raw[0:kk]                     #清空雷达图信息

    #对于图像的callback，将图像信息保存至image_raw列表中
    def image_callback(self, msg):
        #self.image_append(msg) originL version
        self.image_raw.append(msg)

if __name__ == "__main__":
    # 初始化一个名为fusion的节点，并且规定以后若创建了同名节点，按序号排列
    rospy.init_node("fusion", anonymous = True)
    # 初始化雷达和摄像头信息融合的对象
    Cone_fusion = cone_fusion()
    # 给低通滤波器提供初始的参数
    lp_filter.giveInput(0.0)
    lp_filter.initiallize()
    # 创建两个订阅器，分别接受处理雷达和摄像头的信息
    rospy.Subscriber("/image_processed", imagel_whole, Cone_fusion.image_callback)
    rospy.Subscriber("/lidar_processed", lidar3_whole, Cone_fusion.lidar_callback)
    rospy.spin()
    
