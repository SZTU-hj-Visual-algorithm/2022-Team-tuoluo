/*********************************************************************************

  *Description: 角度解算,移动和陀螺预测
  *Function List:
     1.GetArmorAngle 击打角度获取接口,出入处理后的装甲信息
     2.ComputeRecoup  移动预测
     3.ComTuoluoShootPosition  陀螺检测
     4.ShootAdjust  相机坐标转换到枪口坐标,将y坐标转换成绝对坐标
     5.ComputeShootTime 计算击打时间和弹道缓冲(不考虑空气阻力等影响)
     第1种移动预测方案,使用绝对坐标(已弃用):
     6.SetKF 滤波控制接口
     7.BufferSetFilter缓冲状态接口
     8.FirstFind    首次发现目标,各项初始化
     9.FirstSetFilter   第一次连续滤波,卡尔曼对象初始化
     10.SetSecOrderKF   二阶加速度滤波(弃用)
     第2种移动预测方案,采用角速度(更稳定,对测距精度要求更低,缺点为不符合实际运动规律):
     11.angle_FirstFind 角度滤波初始化
     12.angle_FirstSetFilter    首次角度滤波
     13.setAngleForceParam  设置角度滤波卡尔曼参数
  * Others:
        包含陀螺判断机制和卡尔曼预测
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
