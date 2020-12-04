#include <ros/ros.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "controlcan.h"
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
        ROS_FATAL_STREAM("VCI_InitCAN("<<i<<") succeeded");

        if (!VCI_StartCAN(gDevType, gDevIdx, i))
        {
            ROS_ERROR_STREAM("VCI_StartCAN("<<i<<") failed");
            return 0;
        }
        ROS_FATAL_STREAM("VCI_StartCAN("<<i<<") succeeded");

        if(!VCI_ClearBuffer(gDevType, gDevIdx, i)){
            ROS_ERROR_STREAM("VCI_InitCAN("<<i<<") failed");
            return 0;
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_controller_controller");
    ros::NodeHandle nh;
    /*ROS_INFO_STREAM("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, TxSleep=%d, TxFrames=0x%08x(%d), TxCount=0x%08x(%d)\n",
           gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames, gTxFrames, gTxCount, gTxCount);*/
    if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) {
        ROS_ERROR_STREAM("VCI_OpenDevice failed");
        return 0;
    }
    ROS_FATAL_STREAM("VCI_OpenDevice succeeded");
    initialCan();
    unsigned ID;
    unsigned data[8];
    memset(data,0,sizeof(unsigned)*8);
    VCI_CAN_OBJ *vco;
    VCI_CAN_OBJ *vcoleft;
    VCI_CAN_OBJ *vcoinit;
    vcoleft = (VCI_CAN_OBJ*)malloc(sizeof(VCI_CAN_OBJ));
    vcoinit = (VCI_CAN_OBJ*)malloc(sizeof(VCI_CAN_OBJ));
    vcoleft->ID = 0xa3;
    vcoleft->SendType = 0;// 正常发送
    vcoleft->RemoteFlag = 0;// 数据帧
    vcoleft->ExternFlag = 0;// 标准帧
    vcoleft->DataLen = 8;
    vcoleft->Data[0] = 228;
    vcoleft->Data[1] = 157;
    for(int i=2; i<8; ++i){
        vcoleft->Data[i] = 0x0;
    }
    vcoinit->ID = 0xa2;
    vcoinit->SendType = 0;// 正常发送
    vcoinit->RemoteFlag = 0;// 数据帧
    vcoinit->ExternFlag = 0;// 标准帧
    vcoinit->DataLen = 8;
    vcoinit->Data[0] = 0x01;
    for(int i=1; i<8; ++i){
        vcoinit->Data[i] = 0x0;
    }
    vco = vcoinit;
    /*ROS_INFO_STREAM("%x %d %d %d %d %d %d %d %d\n",
           vco->ID, vco->Data[0], vco->Data[1], vco->Data[2], vco->Data[3], vco->Data[4], vco->Data[5], vco->Data[6], vco->Data[7]);*/
    if(!VCI_Transmit(gDevType, gDevIdx, 0, vcoinit, 1)){
        ROS_ERROR_STREAM("Transmit fail");
        return 0;
    }
    else{
        ROS_FATAL_STREAM("Transmit succeeded");
    }
    free(vco);
    VCI_CloseDevice(gDevType, gDevIdx);
    ROS_FATAL_STREAM("VCI_CloseDevice");
    return 0;
}
