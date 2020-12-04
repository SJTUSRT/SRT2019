#!/usr/bin/python

# -*- coding: UTF-8 -*-

import roslib; #roslib.load_manifest(PKG)
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from numpy import linalg as la
import rospy
import cv2
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 
import rospy
from visualization_msgs.msg import *
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
#该文件创建了一个'fusion_visual'节点发布'/fusion_visual'话题,监听'cone_detected'并发布一个Marker, 以可视化方式展示cone位置与颜色
class Cone_small():
    def __init__(self,iidd,x,y,z,color):
        self.marker_pub = rospy.Publisher("/fusion_visual", Marker, queue_size = 10)
        marker = Marker()
        marker.id = iidd
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://fusion/model_description/meshes/cone.dae"
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = -0.25
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        print color
            #marker.action = Marker.ADD
        if color == "r":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5 
        elif color == "b" or color == 'b' :
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.6
            marker.color.a = 0.5 
            print "here"
        elif color == "y":
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5 
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.5 
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.25)
        self.marker_pub.publish(marker)

def call_back(msg):
    length = len(msg.cones)
    for ii in range(0,length):
        cone_small = Cone_small(ii,msg.cones[ii].x,msg.cones[ii].y,0.15,msg.cones[ii].color)


def main():
    rospy.init_node('fusion_visual', anonymous = True)
    rospy.Subscriber("/cone_detected", cone_pos_whole, call_back, queue_size = 1, buff_size = 52428800)
    rospy.spin()
    
if __name__ == "__main__":
    main()
