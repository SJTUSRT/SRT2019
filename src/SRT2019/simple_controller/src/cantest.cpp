#include <ros/ros.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "controlcan.h"
#include <geometry_msgs/Twist.h>
#include <simple_controller/Srt_Control.h>
#define msleep(ms)  usleep((ms)*1000)
#define min(a,b)  (((a) < (b)) ? (a) : (b))

#define MAX_CHANNELS  4
#define CHECK_POINT  200
#define RX_WAIT_TIME  100
#define RX_BUFF_SIZE  1000

unsigned gDevType = 4;
unsigned gDevIdx = 0;
unsigned gChMask = 3;
unsigned gBaud = 0x1c00;
unsigned gTxType = 0;
unsigned gTxSleep = 3;
unsigned gTxFrames = 10;
unsigned gTxCount = 10;
bool is_stop = true;
bool is_stop_changed = false;
bool is_controlled = false;
//following are numbers used in function Callback
//transforming coefficeints of steer angle mapping
//const double slope = 934.3333333333;
//const double intercept = 2483;

const double slope = 1026;
const double intercept = 29370;

unsigned s2n(const char *s)
{
    unsigned l = strlen(s);
    unsigned v = 0;
    unsigned h = (l > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'));
    unsigned char c;
    unsigned char t;
    if (!h) return atoi(s);
    if (l > 10) return 0;
    for (s += 2; c = *s; s++)
    {
        if (c >= 'A' && c <= 'F') c += 32;
        if (c >= '0' && c <= '9') t = c - '0';
        else if (c >= 'a' && c <= 'f') t = c - 'a' + 10;
        else return 0;
        v = (v << 4) | t;
    }
    return v;
}

int initialCan(){
    VCI_INIT_CONFIG config;
    config.AccCode = 0x18A00000;
    config.AccMask = 0x001FFFFF;
    config.Filter = 1;
    config.Mode = 0;
    config.Timing0 = 0x00;
    config.Timing1 = 0x1C;

    int i, j;
    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;

        if (!VCI_InitCAN(gDevType, gDevIdx, i, &config))
        {
            ROS_ERROR_STREAM("VCI_InitCAN("<<i<<") failed");
            return 0;
        }
        ROS_INFO_STREAM("VCI_InitCAN("<<i<<") succeeded");

        if (!VCI_StartCAN(gDevType, gDevIdx, i))
        {
            ROS_ERROR_STREAM("VCI_StartCAN("<<i<<") failed");
            return 0;
        }
        ROS_INFO_STREAM("VCI_StartCAN("<<i<<") succeeded");

        if(!VCI_ClearBuffer(gDevType, gDevIdx, i)){
            ROS_ERROR_STREAM("VCI_InitCAN("<<i<<") failed");
            return 0;
        }
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "simple_controller_controller");
    ros::NodeHandle nh;
    ROS_INFO("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, TxSleep=%d, TxFrames=0x%08x(%d), TxCount=0x%08x(%d)\n",
             gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames, gTxFrames, gTxCount, gTxCount);
    if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) {
        ROS_ERROR_STREAM("VCI_OpenDevice failed");
        return 0;
    }
    ROS_INFO_STREAM("VCI_OpenDevice succeeded");
    initialCan();
    unsigned ID;
    unsigned data[8];
    memset(data,0,sizeof(unsigned)*8);
    VCI_CAN_OBJ vcoinit[3];
    vcoinit[0].ID = 0x02;
    vcoinit[0].SendType = 0;// 正常发送
    vcoinit[0].RemoteFlag = 0;// 数据帧
    vcoinit[0].ExternFlag = 0;// 标准帧
    vcoinit[0].DataLen = 8;
    for(int i=0; i<8; ++i){
        vcoinit[0].Data[i] = 0x0;
    }
    vcoinit[1].ID = 0x01;
    vcoinit[1].SendType = 0;// 正常发送
    vcoinit[1].RemoteFlag = 0;// 数据帧
    vcoinit[1].ExternFlag = 0;// 标准帧
    vcoinit[1].DataLen = 8;
    vcoinit[1].Data[0] = 0x00;
    vcoinit[1].Data[1] = 0x00;
    for(int i=2; i<8; ++i){
        vcoinit[1].Data[i] = 0x0;
    }
    vcoinit[2].ID = 0x03;
    vcoinit[2].SendType = 0;// 正常发送
    vcoinit[2].RemoteFlag = 0;// 数据帧
    vcoinit[2].ExternFlag = 0;// 标准帧
    vcoinit[2].DataLen = 8;
    vcoinit[2].Data[0] = 0x00;
    vcoinit[2].Data[1] = 0x01;
    for(int i=2; i<8; ++i){
        vcoinit[2].Data[i] = 0x0;
    }
    if(!VCI_Transmit(gDevType, gDevIdx, 0, vcoinit, 3)){
        ROS_ERROR_STREAM("Transmit fail");
        return 0;
    }
    else{
        ROS_INFO_STREAM("Transmit succeeded");
    }
    while(ros::ok()){
        VCI_CAN_OBJ vco[1];
        vco[0].ID = 0x01;
        vco[0].SendType = 0;// 正常发送
        vco[0].RemoteFlag = 0;// 数据帧
        vco[0].ExternFlag = 0;// 标准帧
        vco[0].DataLen = 8;
        vco[0].Data[0]=0x01;
        vco[0].Data[1]=200;
        for(int i=2; i<8; ++i)
            vco[0].Data[i] = 0x0;
        if(!VCI_Transmit(gDevType, gDevIdx, 0, vco, 1)){
            ROS_ERROR_STREAM("Transmit fail");
            return;
        }
        else{
            ROS_INFO_STREAM("Transmit success");
            return;
        }
    }
    VCI_CloseDevice(gDevType, gDevIdx);
    ROS_INFO_STREAM("VCI_CloseDevice");
    return 0;
}
