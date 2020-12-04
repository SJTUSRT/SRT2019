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

transf_x = 0.69
transf_y = 0.0
transf_z = -0.55
Threshold = 100
Dist_move = 0.6
L = 1.0#wheelbase
Ld = 2.0#the foresee distance
Stop_coef = 12

def Euc_dist(pos_1,pos_2):
    return la.norm(np.subtract(pos_1,pos_2))


class cone_on_img():
    def __init__(self, u, v):
        self.u = u
        self.v = v

class cone_fusion():
    def __init__(self):
        self.lidar_raw = []
        self.image_raw = []
        self.previous_points = []
        self.previous_target = []
        self.use_previous = 0
        self.stop = 0
        self.fusion_pub = rospy.Publisher("/cone_detected", cone_pos_whole, queue_size = 10)
        self.target_pub = rospy.Publisher("/target_point",cone_pos,queue_size = 10)
        self.control_pub = rospy.Publisher("/srt_control",Srt_Control,queue_size = 10)

    def findMinI(self,l):
        length = len(l)
        mins = 0
        mini = l[0]

        for ii in range(1, length):
            if mini > l[ii]:
                mini = l[ii]
                mins = ii

        return mins 
       
    def match(self, u, v):
        num = len(self.image_raw) - 1
        image_len = len(self.image_raw[num].image_points)
        for ii in range(0, image_len):
            print(u, v)
            print(self.image_raw[num].image_points[ii].start_x, self.image_raw[num].image_points[ii].end_x, self.image_raw[num].image_points[ii].start_y, self.image_raw[num].image_points[ii].end_y)
            if (v > self.image_raw[num].image_points[ii].start_x - Threshold) and (v < self.image_raw[num].image_points[ii].end_x + Threshold) and (u > self.image_raw[num].image_points[ii].start_y - Threshold) and (u < self.image_raw[num].image_points[ii].end_y + Threshold):
                print ("color == ")
                print (self.image_raw[num].image_points[ii].color)
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
                cone = cone_pos()
                cone.x = pose.x
                cone.y = pose.y
                #cone.color = self.image_raw[num].image_points[ii].color
                cone.color = self.match(u, v)
                self.lidar_raw.append(cone)

    def driving(self,target):
        sin_alpha = target[1] / Ld
        steer_angle = math.degrees(math.atan2(2*L*sin_alpha/Ld,1) )
        linear_velocity = 0.5
        if self.use_previous > Stop_coef or (target[0] == 0 and target[1] == 0) or self.stop == 1: 
            steer_angle = 0.0
            linear_velocity = 0.0
        control_param = Srt_Control()
        control_param.linear_velocity = linear_velocity
        control_param.steer_angle = steer_angle
        self.control_pub.publish(control_param)

    def lidar_callback(self, msg):
        msgl = len(msg.lidar_points)
        if msgl != 0:
            for i in msg.lidar_points:
                i.x += transf_x
                i.y += transf_y
                i.z += transf_z
            time_stamp = msg.stamp
            jj = len(self.image_raw) - 1
            delta_t = 1.0
            while jj >= 0:
                if time_stamp > self.image_raw[jj].stamp:
                    delta_t = time_stamp - self.image_raw[jj].stamp
                    break
                jj = jj - 1
            del self.image_raw[0:jj] #the jjth item is not deleted, such that next time we can find at least one ealier time stamp

            length_msg = len(msg.lidar_points)
            for ii in range(0, length_msg):
                self.fusion(msg.lidar_points[ii],delta_t)

            kk = len(self.lidar_raw)#originally: kk = len(self.lidar_raw) - 1
            length_prev = len(self.previous_points)
            for jj in range(0,kk):
                if self.lidar_raw[jj].color == "unknown":
                    for ll in range(0,length_prev):
                        if  Euc_dist(np.array([[self.lidar_raw[jj].x],[self.lidar_raw[jj].y]]),np.array([[self.previous_points[ll].x],[self.previous_points[ll].y]])) < Dist_move:
                            print "change current"
                            print self.lidar_raw[jj]
                            print "to"
                            print self.previous_points[ll]
                            self.lidar_raw[jj].color = self.previous_points[ll].color
                            break
            self.previous_points = []
            for ii in range(0,kk):
                self.previous_points.append(self.lidar_raw[ii])
                if self.lidar_raw[ii].color == 'y':
                    self.stop = 1
            cones_whole = cone_pos_whole(self.lidar_raw, time_stamp)
            self.fusion_pub.publish(cones_whole)
            print(cones_whole)

            target_point = cone_pos()
            target = get_target.get_target(self.lidar_raw)
            print "target"
            print target
            if target[0] != -1:
                self.use_previous = 0
                self.previous_target = target
            else:
                self.use_previous = self.use_previous + 1
                target = self.previous_target
            target_point.x = target[0]
            target_point.y = target[1]
            target_point.color ="unknown"
            self.target_pub.publish(target_point)
            self.driving(target)
            

            del self.lidar_raw[0:kk]
            
    def image_callback(self, msg):
        #self.image_append(msg) originL version
        self.image_raw.append(msg)

if __name__ == "__main__":
    rospy.init_node("fusion", anonymous = True)
    Cone_fusion = cone_fusion()
    rospy.Subscriber("/image_processed", imagel_whole, Cone_fusion.image_callback)
    rospy.Subscriber("/lidar_processed", lidar3_whole, Cone_fusion.lidar_callback)
    rospy.spin()
    
