#include <ros/ros.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <ros/duration.h>
#include <simple_controller/Srt_Control.h>
#include "controlcan.h"
#define msleep(ms) usleep((ms)*1000)
#define min(a,b)  (((a) < (b)) ? (a) : (b))

#define MAX_CHANNELS 4
#define CHECK_POINT 200
#define RX_WAIT_TIME 100
#define RX_BUFF_SIZE 1000
#define DATALEN 8

#define MAX_VEL_INTERVAL 3   // when dv > this number, the accelerating and decelerating rate would be limited.
#define MAX_TORQUE      135
#define ZERO_TORQUE     128

unsigned gDevType = 4;
unsigned gDevIdx = 0;
unsigned gChMask = 3;
unsigned gBaud = 0x1c00;
unsigned gTxType = 0;
unsigned gTxSleep = 3;
unsigned gTxFrames = 10;
unsigned gTxCount = 10;

//duration used in ABS module for controlling sleep time
ros::Duration sleeptime(0, 200000000); //200000000 ms, 0.2 s

//used to judge if the programme should enter the ABS module
bool if_ABS = false;

//following are numbers used in function Callback
//transforming coefficeints of steer angle mapping
const double slope = 934.3333333333;
const double intercept = 2483;

//storing the delta velocity (objective velocity minus current velocity).
double dv = 0;

//the minimum of velocity interval between two instructions.
const double min_inter_vel = 0.1;

//storing the current velocity of the last instruction.
double cur_vel = 0;

//converting a string which contains a memory addtress to an integer.
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
}   //result: when s satisfies the condition, if s isn't a hexadecimal number, directly convert s, else convert s into a decimal. Otherwise it would return 0 (failure).

int initialCan()    //Initializing the CAN device
{
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

void Callback(const simple_controller::Srt_Control &msg)    //Updating the angle with the information received.
{
    unsigned ID;
    unsigned data[8];
    double obj_vel = msg.linear_velocity;   //storing the objective velocity
    double obj_ang = msg.steer_angle;       //storing the objective angle
    if (obj_vel <= 0.001) obj_vel = 0.0;
    if (obj_vel > 5) obj_vel = 5.0;
    if (obj_ang < -30) obj_ang = -30.0;
    if (obj_ang > -30) obj_ang = 30.0;

    unsigned cur_vel_h;     //cuz the current speed reveived is a hex number and is devided into 2 bytes
    unsigned cur_vel_l;
    unsigned int angle;//transformed objective steer angle, from 9,179(right,2483) to 228,157(left, 58525).
    unsigned angle_high;    //cuz the objective angle is a hex number and is devided into 2 bytes
    unsigned angle_low;
    angle = (unsigned int)((slope * (obj_ang + 30)) + intercept);
    angle_high = angle >> 8;
    angle_low = angle - (angle_high << 8);
    memset(data,0,sizeof(unsigned)*8);
    VCI_CAN_OBJ *vco;                       //defining the message which would be transimitted.
    vco = (VCI_CAN_OBJ*)malloc(sizeof(VCI_CAN_OBJ));
    vco->ID = 0xa3;
    vco->SendType = 0;  //Sent successfully
    vco->RemoteFlag = 0;    //Data frame
    vco->ExternFlag = 0;    //Standard frame
    vco->DataLen = DATALEN;
    vco->Data[0] = angle_high;
    vco->Data[1] = angle_low;

    for(int i = DATALEN - 2; i < DATALEN; ++i){     //Setting the rest of Data vector to NULL address.
        vco->Data[i] = 0x0;
    }

    VCI_CAN_OBJ *rvco;                      //defining the feedback message which would be received
    rvco = (VCI_CAN_OBJ*)malloc(sizeof(VCI_CAN_OBJ));
    if(!VCI_Receive(gDevType, gDevIdx, 0, rvco, 1, 100)){
        ROS_ERROR_STREAM("Receiving current status failed");
        return;
    }
    else{
        ROS_FATAL_STREAM("Receiving current status succeeded");
    }

    //calculating the current velocity with unit meter per second
    cur_vel_h = rvco->Data[0];
    cur_vel_l = rvco->Data[1];
    cur_vel = cur_vel_h << 8;
    cur_vel /= 1000.0;

    //calculating the delta velocity
    dv = obj_vel - cur_vel;

    //calculating output torque based on delta velocity
    if (dv > 0){
        if_ABS = false;
        vco->Data[3] = 00;
        vco->Data[4] = 01;
        if (dv > MAX_VEL_INTERVAL){
            vco->Data[2] = MAX_TORQUE;
        }
        else{
            vco->Data[2] = ZERO_TORQUE + (dv / MAX_VEL_INTERVAL) * MAX_TORQUE;
        }
    }
    else{
        vco->Data[2] = ZERO_TORQUE;
        if (cur_vel <= 0.0001){     //considering the error of float number, if current velocity is 0, this operation should be done
            vco->Data[3] = 01;
            vco->Data[4] = 01;
            if_ABS = false;
        }
        else{
            vco->Data[3] = 01;
            vco->Data[4] = 01;
            if_ABS = true;
        }
    }

    if (!VCI_Transmit(gDevType, gDevIdx, 0, vco, 1)){
        ROS_ERROR_STREAM("Transmit failed");
        return;
    }
    else{
        ROS_FATAL_STREAM("Transmit succeeded");
    }

    //ABS module
    if(if_ABS){
        sleeptime.sleep();
        vco->Data[3] = 00;
        VCI_Transmit(gDevType, gDevIdx, 0, vco, 1);
        if_ABS = false;
    }

    free(vco);
    free(rvco);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_controller_controller");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/srt_control", 1, Callback);
    ROS_INFO("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, TxSleep=%d, TxFrames=0x%08x(%d), TxCount=0x%08x(%d)\n",
            gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep,gTxFrames, gTxFrames, gTxCount, gTxCount);
    if (!VCI_OpenDevice(gDevType, gDevIdx, 0)){
        ROS_ERROR_STREAM("VCI_OpenDevice failed");
        return 0;
    }
    ROS_FATAL_STREAM("VCI_OpenDevice succeeded");
    initialCan();
    unsigned ID;
    unsigned data[8];
    memset(data,0,sizeof(unsigned)*8);
    VCI_CAN_OBJ *vcoinit;      //initial signal
    //allocating memory for the object
    vcoinit = (VCI_CAN_OBJ*)malloc(sizeof(VCI_CAN_OBJ));
    //setting initial object's values
    vcoinit->ID = 0xa2;    //means it is a status control instruction
    vcoinit->SendType = 0;  //Sent successfully
    vcoinit->RemoteFlag = 0;    //Data frame
    vcoinit->ExternFlag = 0;    //Standard frame
    vcoinit->DataLen = DATALEN;
    vcoinit->Data[0] = 0x01;

    for(int i=1; i<DATALEN; ++i){
        vcoinit->Data[i] = 0x0;
    }
    if(!VCI_Transmit(gDevType, gDevIdx, 0, vcoinit, 1)){
        ROS_ERROR_STREAM("Transmit fail");
        return 0;
    }
    else{
        ROS_FATAL_STREAM("Transmit succeeded");
    }
    //starting to reveive msg and control
    ros::spin();
    free(vcoinit);
    VCI_CloseDevice(gDevType, gDevIdx);
    ROS_FATAL_STREAM("VCI_CloseDevice");
    return 0;
}
