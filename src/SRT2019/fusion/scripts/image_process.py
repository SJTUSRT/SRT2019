#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import absolute_import, division, print_function
import os
import sys
import cv2
import tensorflow as tf
import numpy as np
import h5py
import rospy
import cone_color_detection as detection

from fusion.msg import imagel_single
from fusion.msg import imagel_whole
#from std_msgs.msg import Image
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from saved_model_predictor_DL import SavedModelPredictor
from detection import sjtu_detection

#该文件创建了一个'image_process'节点和一个'image_processed'话题, 该节点接受相机拍到的图片,利用detection.py中的方法识别出图中的锥桶,
# 再截出每个锥桶bounded_box的坐标, 以imagel_whole的消息发布

#mydetctor对象, 用来处理图片并发布imagel_whole消息
class mydetector():
    #创建'/image_processes'话题
    def __init__(self):
        self.cvb=CvBridge()      
        self.detector=sjtu_detection()
        self.pub = rospy.Publisher('/image_processed',imagel_whole,queue_size = 10)

        
        '''
        rospy.Subscriber('images', Image, self.callback)
        self.pub=rospy.Publisher('has_red_light', Bool, queue_size=1)
        rospy.init_node('traffic_light_detection', anonymous=True)
        '''  

    #函数名: callback
    #功能: 监听'/mynteye/left/image_color', 对收到的Image msg, 利用detection.py中的方法识别出锥桶的bounded_box的坐标, 发送给'image_processed话题'
    #输入: imgmsg: Image格式的msg
    #输出: 向'image_processed'发送imagel_whole消息
    def callback(self, imgmsg):
        msg_out = imagel_whole()
        msg_out.stamp = rospy.get_time()
        img = self.cvb.imgmsg_to_cv2(imgmsg)  
        res = self.detector.predict(img) #对图片中物品进行探测的结果
        cropped_cones = self.detector.get_cropped(res) #识别出的锥桶的bounded_box的坐标(list, 元素为 [start_x,start_y,end_x,end_y])
        #print(cropped_cones)
        length = len(cropped_cones)
        #print (len(self.img))
        print (length)
        for ii in range(0,length):
            y1 = int(cropped_cones[ii][0])
            y2 = int(cropped_cones[ii][2])
            x1 = int(cropped_cones[ii][1])
            x2 = int(cropped_cones[ii][3])
            #print (y1,y2,x1,x2)
            cropped = img[y1:y2,x1:x2] #单个锥桶的bounded_box的图片
            #print (cropped)
            print (len(img))
            color = detection.color_detection(cropped)
            #print ("cone", ii)
            #print (color)
            temp_cone = imagel_single()
            temp_cone.start_x = x1
            temp_cone.start_y = y1
            temp_cone.end_x = x2
            temp_cone.end_y = y2
            temp_cone.color = color
            msg_out.image_points.append(temp_cone)
            font = cv2.FONT_HERSHEY_SIMPLEX
            if color == 'r':
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
                #cv2.putText(img, detection_class+str(detection_score), (start_y, start_x), font, 1, (0, 0, 255), 2)
            elif color == 'b':
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
                #cv2.putText(img, detection_class+str(detection_score), (start_y, start_x), font, 1, (255, 0, 0), 2)
            elif color == 'y':
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 3)
                #cv2.putText(img, detection_class+str(detection_score), (start_y, start_x), font, 1, (0, 255, 255), 2)
            else:
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), 3)
                #cv2.putText(self.img, detection_class+str(detection_score), (x1, y1), (x2, y2), font, 1, (0, 0, 0), 2)
        #outimg = self.detector.visualize(self.img,res)
        cv2.imshow("cone",img)
        cv2.waitKey(1)
        self.pub.publish(msg_out)


if __name__ == "__main__":
    rospy.init_node('image_process',anonymous = True)
    detector = mydetector();
    rospy.Subscriber('/mynteye/left/image_color',Image,detector.callback,queue_size = 1, buff_size = 52428800)
    rospy.spin()

