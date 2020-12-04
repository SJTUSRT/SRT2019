#使用小孔成像法实现将空间中三维坐标点映射到二维坐标中

import numpy as np

def lidar2img(ldx, ldy,ldz):
    #REQUIRES: ldx,ldy,ldz is the coordinates in the carema plane, the image plane is vertical to x axis
    #EFFECTS: return the two dimensional coordinates in the carema plane
    #fx,fy和相机的焦距，像素的大小有关
    #cx,cy是平移的距离，和相机成像平面的大小有关
    #fx cx in the y-axis
    #fy cy in the z-axis
    fx=713.74560546875000000
    fy=714.40148925781250000
    cx=632.15820312500000000
    cy=350.23068237304687500
    #deviation is the length between two eyes in meter
    #fix ldy to be the coordiates in left eye
    deviation=0.12071619415283203125
    ldy=ldy-deviation/2;
    #matrix称为相机的内参数矩阵
    matrix=np.array([[-fx,0,cx],[0,-fy,cy],[0,0,1]])
    positionmatrix=np.array([ldy,ldz,ldx])
    ansmatrix=np.dot(matrix,positionmatrix)
    #ldv in the y-axis
    #ldu in the z-axis
    #ldv和ldu为所求的二维坐标
    ldv=ansmatrix[0]/ldx;
    ldu=ansmatrix[1]/ldx;
    return ldu,ldv


