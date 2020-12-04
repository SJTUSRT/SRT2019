#!/usr/bin/python

# -*- coding: UTF-8 -*-

import rospy
import math
import numpy as np
from numpy import linalg
from fusion.msg import cone_pos_whole
from fusion.msg import map_points
from fusion.msg import cone_self_pos
from fusion.msg import IMU
from fusion.msg import cone_pos

THRESHOLD = 0.3

class imu():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.sin_yaw = 0.0
        self.cos_yaw = 1.0
        self.v_imu_x = 0.0
        self.v_imu_y = 0.0
        self.stamp = 0.0

car_pos = imu()
car_pos.x = 0.0
car_pos.y = 0.0
car_pos.sin_yaw = 0.0
car_pos.cos_yaw = 1.0
car_pos.v_imu_x = 0.0
car_pos.v_imu_y = 0.0
car_pos.stamp = 0.0
 
class without_imu_points():
    def __init__(self):
        self.left_determined = []
        self.right_determined = []
        self.self_pose = []
        self.cones_pose = []
        self.cones_pose_tmp = []
        self.round = 0
        self.stamp = 0.0

    def map_dist(self, map_point1, map_point2):
        return math.sqrt(math.pow(map_point1.mean_x - map_point2.mean_x, 2) + math.pow(map_point1.mean_y - map_point2.mean_y, 2))


    def cones_determined_append(self, cone):
        if cone.color == 'r':
            len_r_cones = len(self.left_determined)
            if len_r_cones == 0:
                self.left_determined.append(cone)
            else:
                flag = 0
                for ii in range(0, len_r_cones):
                    if self.map_dist(self.left_determined[ii], cone) < THRESHOLD:
                        self.left_determined[ii].mean_x = ((self.left_determined[ii].update_time - 2) * self.left_determined[ii].mean_x + cone.mean_x)/ (self.left_determined[ii].update_time - 1)
                        self.left_determined[ii].mean_y = ((self.left_determined[ii].update_time - 2) * self.left_determined[ii].mean_y + cone.mean_y)/ (self.left_determined[ii].update_time - 1)
                        self.left_determined[ii].update_round = cone.update_round
                        self.left_determined[ii].update_time = cone.update_time - 2 #
                        flag = 1
                if flag == 0:
                    self.left_determined.append(cone)
        if cone.color == 'b':
            len_b_cones = len(self.right_determined)
            if len_b_cones == 0:
                self.right_determined.append(cone)
            else:
                flag = 0
                for ii in range(0, len_b_cones):
                    if self.map_dist(self.right_determined[ii], cone) < THRESHOLD:
                        self.right_determined[ii].mean_x = ((self.right_determined[ii].update_time - 2) * self.right_determined[ii].mean_x + cone.mean_x)/ (self.right_determined[ii].update_time - 1)
                        self.right_determined[ii].mean_y = ((self.right_determined[ii].update_time - 2) * self.right_determined[ii].mean_y + cone.mean_y)/ (self.right_determined[ii].update_time - 1)
                        self.right_determined[ii].update_round = cone.update_round
                        self.right_determined[ii].update_time = cone.update_time - 2 #
                        flag = 1
                if flag == 0:
                    self.right_determined.append(cone)

    def cones_trans(self):
        len_cones_pose = len(self.cones_pose)
        ii = 0
        cone_tmp = map_points() 
        while ii < len_cones_pose:
            if self.cones_pose[ii].update_round == self.round:
                cone_pose_detected = np.array([[self.cones_pose[ii].x2], [self.cones_pose[ii].y2]])
                self_position = np.array([[car_pos.x], [car_pos.y]])
                yaw_trans = np.array([[car_pos.cos_yaw, -car_pos.sin_yaw], [car_pos.sin_yaw, car_pos.cos_yaw]])
                cone_pose_map = self_position + np.dot(yaw_trans, cone_pose_detected)
                cone_tmp.mean_x = cone_pose_map[0][0]
                cone_tmp.mean_y = cone_pose_map[1][0]
                cone_tmp.update_round = self.cones_pose[ii].update_round
                cone_tmp.update_time = self.cones_pose[ii].update_time
                cone_tmp.color = self.cones_pose[ii].color
                self.cones_determined_append(cone_tmp)
            ii += 1

    def choose_two_cones(self):
        len_cones_pose = len(self.cones_pose)
        
        ii = 0
        flag = 0
        for ii in range(0, len_cones_pose):
            if self.cones_pose[ii].update_round == self.round and flag == 0:
                cone_tmp1 = self.cones_pose[ii]
                flag += 1
                continue
            if self.cones_pose[ii].update_round == self.round and flag == 1:
                cone_tmp2 = self.cones_pose[ii]
                flag += 1
                break
            
        if flag == 2:
            return cone_tmp1, cone_tmp2
        else:
            return 0, 0    

    def self_pose_caculate(self):
        
        cone1, cone2 = self.choose_two_cones()
        len_self_pose = len(self.self_pose)
        if len_self_pose > 1:
            if self.self_pose[len_self_pose - 1].x == 0.0 and self.self_pose[len_self_pose - 1].y == 0.0 and self.self_pose[len_self_pose - 2].x == 0.0 and self.self_pose[len_self_pose - 2].y == 0.0:
                if cone1 != 0:
                    former_pos = self.self_pose[len(self.self_pose) - 1]
                    a = np.array([[cone1.x2 - cone2.x2, -(cone1.y2 - cone2.y2)], [(cone1.y2 - cone2.y2), cone1.x2 - cone2.x2]])
                    print(cone1)
                    print(cone2)
                    b = np.array([[former_pos.cos_yaw * (cone1.x1 - cone2.x1) - former_pos.sin_yaw * (cone1.y1 - cone2.y1), -(cone1.y2 - cone2.y2)], [former_pos.sin_yaw * (cone1.x1 - cone2.x1) + former_pos.cos_yaw * (cone1.y1 - cone2.y1), cone1.x2 - cone2.x2]])
                    c = np.array([[cone1.x2 - cone2.x2, former_pos.cos_yaw * (cone1.x1 - cone2.x1) - former_pos.sin_yaw * (cone1.y1 - cone2.y1)], [cone1.y2 - cone2.y2, former_pos.sin_yaw * (cone1.x1 - cone2.x1) + former_pos.cos_yaw * (cone1.y1 - cone2.y1)]])
                    car_pos.sin_yaw = np.linalg.det(b) / np.linalg.det(a)
                    car_pos.cos_yaw = np.linalg.det(c) / np.linalg.det(a)
                    car_pos.x = former_pos.x + cone1.x1 * former_pos.cos_yaw - cone1.y1 * former_pos.sin_yaw - cone1.x2 * car_pos.cos_yaw + cone1.y2 * car_pos.cos_yaw
                    car_pos.y = former_pos.y + cone1.x1 * former_pos.sin_yaw + cone1.y1 * former_pos.cos_yaw - cone1.y2 * car_pos.sin_yaw - cone1.x2 * car_pos.cos_yaw
            else:
                if cone1 != 0:
                    former_pos = self.self_pose[len(self.self_pose) - 1]
                    a = np.array([[cone1.x2 - cone2.x2, -(cone1.y2 - cone2.y2)], [(cone1.y2 - cone2.y2), cone1.x2 - cone2.x2]])
                    b = np.array([[former_pos.cos_yaw * (cone1.x1 - cone2.x1) - former_pos.sin_yaw * (cone1.y1 - cone2.y1), -(cone1.y2 - cone2.y2)], [former_pos.sin_yaw * (cone1.x1 - cone2.x1) + former_pos.cos_yaw * (cone1.y1 - cone2.y1), cone1.x2 - cone2.x2]])
                    c = np.array([[cone1.x2 - cone2.x2, former_pos.cos_yaw * (cone1.x1 - cone2.x1) - former_pos.sin_yaw * (cone1.y1 - cone2.y1)], [cone1.y2 - cone2.y2, former_pos.sin_yaw * (cone1.x1 - cone2.x1) + former_pos.cos_yaw * (cone1.y1 - cone2.y1)]])
                    car_pos.sin_yaw = np.linalg.det(b) / np.linalg.det(a)
                    car_pos.cos_yaw = np.linalg.det(c) / np.linalg.det(a)
                    car_pos.x = former_pos.x + cone1.x1 * former_pos.cos_yaw - cone1.y1 * former_pos.sin_yaw - cone1.x2 * car_pos.cos_yaw + cone1.y2 * car_pos.cos_yaw
                    car_pos.y = former_pos.y + cone1.x1 * former_pos.sin_yaw + cone1.y1 * former_pos.cos_yaw - cone1.y2 * car_pos.sin_yaw - cone1.x2 * car_pos.cos_yaw
                else:
                    v_x = (self.self_pose[len_self_pose - 1].x - self.self_pose[len_self_pose - 2].x) / (self.self_pose[len_self_pose - 1].stamp - self.self_pose[len_self_pose - 2].stamp)
                    v_y = (self.self_pose[len_self_pose - 1].y - self.self_pose[len_self_pose - 2].y) / (self.self_pose[len_self_pose - 1].stamp - self.self_pose[len_self_pose - 2].stamp)
                    car_pos.x = car_pos.x + v_x * (self.stamp - self.self_pose[len_self_pose - 1].stamp)
                    car_pos.y = car_pos.y + v_y * (self.stamp - self.self_pose[len_self_pose - 1].stamp)
                    car_pos.stamp = self.stamp
        print(car_pos.x, car_pos.y)

    def cones_pose_append(self, cone_posi):
        
        len_cones_pose = len(self.cones_pose)
        
        if len_cones_pose == 0:
            self.cones_pose.append(cone_posi)
        else:
            flag = 0
            for ii in range(0, len_cones_pose): 
                if self.Euc_dist(self.cones_pose[ii], cone_posi) < THRESHOLD and cone_posi.update_round - self.cones_pose[ii].update_round < 2:
                    self.cone_info_update(self.cones_pose[ii], cone_posi)
                    flag = 1
            if flag == 0:
                self.cones_pose.append(cone_posi)
        

    def dist(self, cone_pose):
        if cone_pose.x2 == 0 and cone_pose.y2 == 0:
            return math.sqrt(math.pow(cone_pose.x1, 2) + math.pow(cone_pose.y1, 2))
        else:
            return math.sqrt(math.pow(cone_pose.x2, 2) + math.pow(cone_pose.y2, 2))

    def cones_pose_tmp_correct(self):
        len_tmp = len(self.cones_pose_tmp)
        if len_tmp > 0:
            for ii in range(0, len_tmp):
                if self.cones_pose_tmp[ii].update_time < 2 and self.round - self.cones_pose_tmp[ii].update_round > 3: #del mis_points
                    self.cones_pose_tmp[ii] = 0
            #flag = 1
            ii = 0
            while ii < len_tmp:
                if self.cones_pose_tmp[ii] == 0:
                    self.cones_pose_tmp.remove(0)
                    #flag = 1
                    len_tmp -= 1
                else:
                    ii += 1
                        
            
            len_tmp = len(self.cones_pose_tmp)#append the ordered cone_points we need
            if len_tmp > 0:
                last_mini = 0.0
                mini = cone_self_pos()
                for ll in range(0, len_tmp):
                    for jj in range(0 , len_tmp):
                        if self.cones_pose_tmp[jj].update_time >= 2 and self.cones_pose_tmp[jj].update_round == self.round and self.dist(self.cones_pose_tmp[jj]) > last_mini:
                            kk = jj + 1
                            while kk < len_tmp:
                                if self.cones_pose_tmp[kk].update_time >= 2 and self.cones_pose_tmp[kk].update_round == self.round and self.dist(self.cones_pose_tmp[kk]) > last_mini:
                                    if self.dist(self.cones_pose_tmp[jj]) < self.dist(self.cones_pose_tmp[kk]):
                                        mini = self.cones_pose_tmp[jj]
                                    else:
                                        mini = self.cones_pose_tmp[kk]
                                kk += 1
                            self.cones_pose_append(mini)
                            last_mini = self.dist(mini)
                            break

    def Euc_dist(self, cone_self_pos1, cone_self_pos2):
        if cone_self_pos1.x2 == 0 and cone_self_pos1.y2 == 0:
            if cone_self_pos2.x2 == 0 and cone_self_pos2.y2 == 0:
                return math.sqrt(math.pow(cone_self_pos1.x1 - cone_self_pos2.x1, 2) + math.pow(cone_self_pos1.y1 - cone_self_pos2.y1, 2))
            else:
                return math.sqrt(math.pow(cone_self_pos1.x1 - cone_self_pos2.x2, 2) + math.pow(cone_self_pos1.y1 - cone_self_pos2.y2, 2))
        else:
            if cone_self_pos2.x2 == 0 and cone_self_pos2.y2 == 0:
                return math.sqrt(math.pow(cone_self_pos1.x2 - cone_self_pos2.x1, 2) + math.pow(cone_self_pos1.y2 - cone_self_pos2.y1, 2))
            else:
                return math.sqrt(math.pow(cone_self_pos1.x2 - cone_self_pos2.x2, 2) + math.pow(cone_self_pos1.y2 - cone_self_pos2.y2, 2))

    def cone_info_update(self, cone_self_pos1, cone_self_pos2):
        #print(cone_self_pos1, cone_self_pos2)
        if cone_self_pos1.x2 == 0 and cone_self_pos1.y2 == 0:
            cone_self_pos1.x2 = cone_self_pos2.x1
            cone_self_pos1.y2 = cone_self_pos2.y1
            cone_self_pos1.stamp2 = cone_self_pos2.stamp1
            cone_self_pos1.color = cone_self_pos2.color    
        else:
            cone_self_pos1.x1 = cone_self_pos1.x2
            cone_self_pos1.y1 = cone_self_pos1.y2
            cone_self_pos1.stamp1 = cone_self_pos1.stamp2
            cone_self_pos1.x2 = cone_self_pos2.x1
            cone_self_pos1.y2 = cone_self_pos2.y1
            cone_self_pos1.stamp2 = cone_self_pos2.stamp1
        cone_self_pos1.color = cone_self_pos2.color
        cone_self_pos1.update_round = cone_self_pos2.update_round
        cone_self_pos1.update_time += 1

    def cones_pose_tmp_append(self, cone_self_pos):
        len_cones_pose_tmp = len(self.cones_pose_tmp)
        #print(len_cones_pose_tmp)
        flag = 0
        if len_cones_pose_tmp == 0:
            self.cones_pose_tmp.append(cone_self_pos)
            flag = 1
        else:
            for ii in range(0, len_cones_pose_tmp):
                if self.Euc_dist(self.cones_pose_tmp[ii], cone_self_pos) < THRESHOLD and cone_self_pos.update_round - self.cones_pose_tmp[ii].update_round == 1 and self.Euc_dist(self.cones_pose_tmp[ii], cone_self_pos) > 0 and (self.cones_pose_tmp[ii].color == cone_self_pos.color or self.cones_pose_tmp[ii].color == "unknown"):
                    self.cone_info_update(self.cones_pose_tmp[ii], cone_self_pos)
                    
                    flag = 1
                    break
                    
        if flag == 0:
            self.cones_pose_tmp.append(cone_self_pos)
                    
                    

    def callback(self, msg):
        self.round += 1
        #self.stamp = msg.stamp
        self.self_pose.append(car_pos) #record car_pos
        #print(car_pos.x, car_pos.y)
        len_cones = len(msg.cones)
        cone_self_pose = cone_self_pos()
        for ii in range(0, len_cones):
            cone_self_pose.x1 = msg.cones[ii].x
            cone_self_pose.y1 = msg.cones[ii].y
            cone_self_pose.stamp1 = msg.stamp
            cone_self_pose.x2 = 0.0
            cone_self_pose.y2 = 0.0
            cone_self_pose.stamp2 = 0.0
            cone_self_pose.update_round = self.round
            cone_self_pose.update_time = 1
            cone_self_pose.color = msg.cones[ii].color
            
            self.cones_pose_tmp_append(cone_self_pose) #get info of cones
        self.cones_pose_tmp_correct()
        
        self.self_pose_caculate() #caculate self_position
        
        self.cones_trans()#transform cone_position
        self.stamp = msg.stamp
        

if __name__ == "__main__":
    rospy.init_node("fusion", anonymous = True)
    Without_imu_points = without_imu_points()
    rospy.Subscriber("/cone_detected", cone_pos_whole, Without_imu_points.callback)
    rospy.spin()



