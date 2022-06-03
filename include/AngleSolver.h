/*********************************************************************************
一些变量的初始化
**********************************************************************************/
#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H


#include <opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
using namespace cv;



class AngleSolver
{
public:
    float fBigArmorWidth;
    float fBigArmorHeight;
    float fSmallArmorWidth;
    float fSmallArmorHeight;        //原5.5cm

    //坐标系转换
    float ptz_camera_y ;                  //相机与云台垂直方向距离差
    float ptz_camera_z;                  //相机与云台轴向方向距离差
    float ptz_camera_x;                    //相机与云台水平方向距离差

    float ptz_camera_pitch ;            //对应绕x旋转角度,弧度，角度乘0.017453
    float ptz_camera_yaw;               //对应绕y旋转角度，弧度
    float ptz_camera_roll ;                 //对应绕z旋转角度，弧度
public:

    AngleSolver();

};
#endif // ANGLESOLVER_H
