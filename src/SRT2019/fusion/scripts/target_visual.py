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

class Cone_small():
    def __init__(self,iidd,x,y):
        print "x = "
        print x
        print "y = "
        print y
        self.marker_pub = rospy.Publisher("/target_visual", Marker, queue_size = 10)
        marker = Marker()
        marker.id = iidd
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = -0.25
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.5
 
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.25)
        self.marker_pub.publish(marker)

def call_back(msg):
    print "time = "
    print rospy.get_time()
    cone_small = Cone_small(1,msg.x,msg.y)


def main():
    rospy.init_node('target_visual', anonymous = True)
    rospy.Subscriber("/target_point", cone_pos, call_back, queue_size = 1, buff_size = 52428800)
    rospy.spin()
    
if __name__ == "__main__":
    main()
