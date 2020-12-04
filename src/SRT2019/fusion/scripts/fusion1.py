import rospy
import lidar2img
from fusion.msg import lidar3_single
from fusion.msg import lidar3_whole
from fusion.msg import imagel_single
from fusion.msg import imagel_whole
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
#该文件创建了一个fusion节点,和一个'cone_detected'话题, 订阅了image_processed和lidar_processed两个话题
#当收到image_processed消息时, 加入image_raw中
#当收到lidar_processed消息时, 对每个锥桶的雷达三维坐标点将其投影到图像平面中, 然后在image_raw中找到最近一次的imagel_whole消息,
# 在其中找到与它距离最近的图片以确定该锥桶的颜色, 并以cos_pos_whole消息发布给'cone_detected'
transf_x = -69.0
transf_y = 0.0
transf_z = 55.0


#锥桶在图中的坐标
class cone_on_img():
    def __init__(self, u, v):
        self.u = u
        self.v = v

# cone_fusion对象用来处理与融合lidar,image
class cone_fusion():
    def __init__(self):
        self.lidar_raw = [] #保存要发布的cone_pos对象
        self.image_raw = [] #保存收到的imagel_whole消息
        self.fusion_pub = rospy.Publisher("/cone_detected", cone_pos_whole, queue_size = 10)

    #函数名:findMinI
    #功能: 通过遍历找到队列中最小元素的编号
    #输入：l:一个list
    #返回值：最小元素的编号
    def findMinI(self,l):
        length = len(l)
        mins = 0
        mini = l[0]

        for ii in range(1, length):
            if mini > l[ii]:
                mini = l[ii]
                mins = ii

        return mins        


    #函数名：Euc_dist
    #功能：计算锥桶三维点投影在图像上的坐标到锥桶图片中心的欧氏距离
    #输入：pos_1:cone_on_img对象 pos_2: imagel_single消息
    #返回值:  锥桶到锥桶图片中心的欧氏距离
    def Euc_dist(self,pos_1,pos_2):
        pos2_x = (pos_2.start_x + pos_2.end_x) / 2
        pos2_y = (pos_2.start_y + pos_2.end_y) / 2
        return ((pos_1.u - pos2_x) ** 2 + (pos_1.v - pos2_y) ** 2) ** 0.5

    #函数名: fusion
    #功能: 对于给定的雷达探测到的锥桶坐标, 在image_raw中寻找与它距离最近的图片, 得到该锥桶的颜色, 生成一个cone_pos对象,
    # 其中坐标来自lidar的pose，颜色来自camera
    #输入: pose: lidar3_single对象(一个三维坐标点)  dt:当前lidar3_whole对象与最晚一个早于它的imagel_whole的时间差
    #输出: 将cone_pos对象加入lidar_raw中
    def fusion(self, pose, dt, msg):
        u, v = lidar2img(pose.x, pose.y, pose.z) #将三维坐标点投影到图像平面
        coneTmp = cone_on_img(u, v)
        jj = len(image_raw)
        dist = []

        if jj != 0:
            for ii in range(0, jj):
                dist.append(Euc_dist(coneTmp, image_raw[jj]))

            ii = findMinI(dist)
            cone = cone_pos()
            cone.x = pose.x
            cone.y = pose.y
            cone.color = image_raw[ii].color
            self.lidar_raw.append(cone)

    #函数名: lidar_callback
    #功能:
    #输入: lidar3_whole消息
    #输出:
    def lidar_callback(self, msg):
        msgl = len(msg.lidar3_points)
        if msgl != 0:
            #坐标平移
            for i in msg.lidar_points:
                i.x += transf_x
                i.y += transf_y
                i.z += transf_z
            time_stamp = msg.stamp
            jj = len(self.image_raw) - 1
            delta_t = 1.0
            #在image_raw里所有早于msg时间的对象中, 保留最后一个, 之前的全删掉
            while jj >= 0:
                if time_stamp > self.image_raw[jj].stamp:
                    delta_t = time_stamp - self.image_raw[jj].stamp
                    break
                jj = jj - 1
            del self.image_raw[0:jj] #the jjth item is not deleted, such that next time we can find at least one ealier time stamp

            #对每个坐标点, 通过fusion确定其颜色, 创建一个cone_pos对象加入self.lidar_raw中
            length_msg = len(msg.lidar3_points)
            for ii in range(0,length_msg):
                self.fusion(msg.lidar3_points[ii],delta_t,msg)
            #发布给'cone_detected'
            kk = len(lidar_raw) - 1
            while kk >= 0:
                self.fusion_pub.publish(self.lidar_raw[kk])
                del self.lidar_raw[kk]
                kk = kk - 1

    #函数名:image_callback
    #功能: 我怀疑该行代码写错了, self.image_raw.append(msg)写成了  self.image_raw.append(msg) 回调函数, 将收到的imagel_whole信息添加到image_row中
    def image_callback(self, msg):
        self.image_append(msg)

if __name__ == "__main__":
    rospy.init_node("fusion", anonymous = True)
    Cone_fusion = cone_fusion()
    rospy.Subscriber("/image_processed", imagel_whole, cone_fusion.image_callback)
    rospy.Subscriber("/lidar_processed", lidar3_whole, cone_fusion.lidar_callback)
    rospy.spin()
    
