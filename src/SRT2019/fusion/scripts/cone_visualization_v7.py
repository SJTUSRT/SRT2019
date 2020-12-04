#!/usr/bin/python

# -*- coding: UTF-8 -*-

import roslib; #roslib.load_manifest(PKG)
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from numpy import linalg as la
from mpl_toolkits.mplot3d import Axes3D
import rospy
import cv2
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 
import rospy
from visualization_msgs.msg import *
from fusion.msg import lidar3_single
from fusion.msg import lidar3_whole



class Cone_detected():
    def __init__(self,bar_list,receive_time):
        self.Cone_pub = rospy.Publisher("/lidar_processed", lidar3_whole, queue_size = 10)
        cone_detected = []
        out_msg = lidar3_whole()
        length = len(bar_list)
        for ii in range(0,length):
            cone_single = lidar3_single()
            cone_single.x = bar_list[ii,0]
            cone_single.y = bar_list[ii,1]
            cone_single.z = bar_list[ii,2]
            cone_detected.append(cone_single)
        out_msg.lidar_points = cone_detected
        out_msg.stamp = receive_time
        self.Cone_pub.publish(out_msg)

def get_height(pt, a, b, c, d):
    #a, b, c, d is the para of ground
    numerator = a * pt[0] + b * pt[1] + c * pt[2] + d
    denominator = np.sqrt(a**2+ b**2 + c**2)
    return numerator/denominator

def is_barrel(pt_3d, a, b, c, d):
    #pt_3d is array
    #Some constant
    RADIUS = 0.3
    MINHEIGHT = 0.2
    MAXHEIGHT = 0.5
    heighest_pt = np.array([0,0,0])
    max_height = 0
    heighest_pt = np.array([0,0,0])
    for sin_pt_3d in pt_3d:
        if sin_pt_3d[2] > max_height:
            max_height = sin_pt_3d[2]
            heighest_pt = sin_pt_3d
    abs_height_top  = get_height(heighest_pt, a, b, c, d)
    avg_x = np.mean(pt_3d[:,0],axis = 0)
    avg_y = np.mean(pt_3d[:,1],axis = 0)
    distance = (pt_3d[0,0] - avg_x)**2 + (pt_3d[0,1] - avg_y)**2
    if pt_3d.shape[1] < 2:
        #If the size of barrier is too little, we can consider it as noise
        return False
    elif abs_height_top < MINHEIGHT or abs_height_top > MAXHEIGHT:
        #Use the height to find barrel
        return False
    elif distance > RADIUS**2:
        #Use the diameter to find barrel
        return False
    else:
        return True
    
def callback(msg):
    receive_time = rospy.get_time()
    pt = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    #define a few constant numbers
    GROUND = -0.36
    DISTANCE = 6.5
    THRESHOLD = 0.2
    GROUND_ERROR = 0.1
    #trans pt(cloud_pt) into list, and filtrate some useless points
    xyz_raw=[]#xyz_raw is the target list
    for single_pt in pt:
        if  single_pt[0]**2 + single_pt[1]**2 + single_pt[2]**2 < DISTANCE**2 and single_pt[0] > 0 and abs(single_pt[1]) < 3.0 and single_pt[2] < 0.5:# The first one filtrate all the points that are far away; the second one filtrate all the points behind the car; the third one filtrate all the points far away on two sides
            xyz_raw.append(list(single_pt))
        else:
            pass
    #Now, xyz_raw is slant, so we should find the representation of the ground
    xyz_raw_array = np.array(xyz_raw)
    #fig2=plt.figure()
    #ax2=Axes3D(fig2)
    #ax2.plot(xyz_raw_array[:,0], xyz_raw_array[:,1], xyz_raw_array[:,2],'r.')
    xyz0 = np.array([np.mean(xyz_raw_array, axis=0)])
    xyz0_rep = np.repeat(xyz0, xyz_raw_array.shape[0], axis=0)
    centered_plane = np.mat(xyz_raw_array - xyz0_rep)
    ind_plane = np.linspace(0,centered_plane.shape[0]-1,500)
    U,S,V = la.svd(centered_plane[ind_plane.astype(int)])
    V = np.array(np.transpose(V))
    #(a,b,c) is normal vector of the ground plane
    a = V[0,2]
    b = V[1,2]
    c = V[2,2]
    #make sure the normal vector point upwards
    if c < 0:
        a = -a
        b = -b
        c = -c
    else:
        pass
    nv_be = np.array([a,b,c])#nv_before is normal vector before rotation
    d = -np.dot(nv_be, xyz0[0])
    #(a b c d) represents ax + by + cz + d = 0
    #now we delete ground
    xyz_near = np.array([[0,0,0]])
    for single_pt in xyz_raw_array:
        if get_height(single_pt, a, b, c, d) > GROUND_ERROR :
            xyz_near = np.vstack((xyz_near, single_pt))
        else:
            pass
    xyz_near = np.delete(xyz_near,0,0)


    #plot here, 3D, without ground
    #fig1=plt.figure()
    #ax=Axes3D(fig1)
    #ax.plot(xyz_near[:,0], xyz_near[:,1], xyz_near[:,2],'b.')
    #end plot


    #every single barrrier is a dict
    single_bar = {'avg_pt': [xyz_near[0,0], xyz_near[0,1], xyz_near[0,2]], 'pts': [0]}
    bar = list([single_bar])
    size_of_pt = xyz_near.shape[0]
    ite_bar = 0
    bar_num = 1
    for ite_all_pt in range(1, size_of_pt):
        if abs(xyz_near[ite_all_pt,0] - xyz_near[ite_all_pt-1,0]) < THRESHOLD and abs(xyz_near[ite_all_pt,1] - xyz_near[ite_all_pt-1,1]) < THRESHOLD:
            bar[ite_bar]['pts'].append(ite_all_pt)
            bar[ite_bar]['avg_pt'] = [np.mean(xyz_near[bar[ite_bar]['pts'], 0],axis = 0), np.mean(xyz_near[bar[ite_bar]['pts'], 1],axis = 0),np.mean(xyz_near[bar[ite_bar]['pts'], 2],axis = 0)]
        else:
            is_in_bar = False
            for i_bar_temp in range(bar_num):
                if abs(xyz_near[ite_all_pt, 0] - bar[i_bar_temp]['avg_pt'][0]) > 3 or abs(xyz_near[ite_all_pt, 1] - bar[i_bar_temp]['avg_pt'][1]) > 3:
                    continue
                else:
                    temp_pt_ind = bar[i_bar_temp]['pts']
                    temp_pt = xyz_near[temp_pt_ind, :]
                    this_pt = xyz_near[ite_all_pt,:]
                    distance = np.sqrt((temp_pt[:,0] - this_pt[0])**2 + (temp_pt[:,1] - this_pt[1])**2)
                    if np.min(distance, axis = 0) < THRESHOLD:
                        is_in_bar = True
                        break
            if is_in_bar:
                bar[i_bar_temp]['pts'].append(ite_all_pt)
                bar[i_bar_temp]['avg_pt'] = [np.mean(xyz_near[bar[i_bar_temp]['pts'], 0],axis = 0), np.mean(xyz_near[bar[i_bar_temp]['pts'], 1],axis = 0),np.mean(xyz_near[bar[ite_bar]['pts'], 2],axis = 0)]
                ite_bar = i_bar_temp
            else:
                bar_num += 1
                ite_bar = bar_num - 1
                bar.append({'avg_pt': [xyz_near[ite_all_pt,0], xyz_near[ite_all_pt,1],xyz_near[ite_all_pt,2]], 'pts': [ite_all_pt]}),
    bar_list = []
    for sin_bar in bar:
        bar_list.append(sin_bar['avg_pt'])
    bar_list = np.array(bar_list)
    for sin_bar in bar:
        if is_barrel(xyz_near[sin_bar['pts'],:],a,b,c,d):
            continue
        else:
            bar.remove(sin_bar)
    bar_list = []
    for sin_bar in bar:
        bar_list.append(sin_bar['avg_pt'])
    bar_list = np.array(bar_list)
    print(bar_list)
    cone_detected = Cone_detected(bar_list,receive_time)

        
def main():
    rospy.init_node('cone_detect', anonymous = True)
    rospy.Subscriber('/rslidar_points', PointCloud2, callback, queue_size = 1, buff_size = 52428800)
    rospy.spin()
    
if __name__ == "__main__":
    main()
