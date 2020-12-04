#include <iostream>
using namespace std;
double *translate(double *a,double depth,int b=3)//a数组为图像点的坐标a[0]为图像的行数即y值，a[1]类比，depth为图像点的深度，b为确定对应模式，不同图像大小有不同参数
{ double cx,cy,fx,fy;//内参矩阵的量，参数来自/MYNT-EYE-D-SDK/src/mynteyed/camera.cc  其中GetStreamIntrinsics函数，应该是出厂的时候标定好的
  switch(b){
    case 1:fx=979.8;fy=942.8;cx=682.3 / 2;cy=254.9;break;//640*480
    case 2:fx=979.8;fy=942.8;cx=682.3;cy=254.9;break;//1280*480
    case 3:fx=979.8;fy=942.8;cx=682.3;cy=254.9*2;break;//1280*720
    case 4:fx=979.8;fy=942.8;cx=682.3*2;cy=254.9*2;break;//2560*720
    default:fx=979.8;fy=942.8;cx=682.3;cy=254.9*2;//1280*720
  }
  double *d=new double[2];
  d[0]=(a[1]-cx)*depth/fx;
  d[1]=(a[0]-cy)*depth/fy;
  return d;
  }

int main(){
  double a[]={6,5};//改改试试
  double depth=2700;//改改试试
  double *d=translate(a,depth,3);
  cout<<d[0]<<d[1]<<endl;
  return 0;}
