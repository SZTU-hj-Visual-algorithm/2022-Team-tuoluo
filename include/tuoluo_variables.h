//
// Created by TanSiwei on 2022/5/14.
//

#ifndef FINDARMOR_TUOLUO_VARIABLES_H
#define FINDARMOR_TUOLUO_VARIABLES_H
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include<math.h>
#include <fstream>
#include<mutex>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml.hpp>

using namespace cv;
using namespace std;

#define PI 3.14159
//#define DEBUG                            //是否进入调试模式
#define IMSHOW                                       //是否显示最终图像
//#define IMAGE_DRAWING                 //是否开启画面绘制

/*
extern string VideoPath;         //视频调试路径
extern string BuffVideoPath;      //大符视频路径
extern string USBDevicPath;                                                   //长焦设备路径
extern string BuffDevicPath ;                                                   //大符底盘相机设备路径
*/

enum pattern{
    FirstFind,        //首次识别
    Shoot,              //连续射击
    stop,               //非连续
    buffering        //缓冲
};


//灯条结构体
struct led{
    RotatedRect box;                                    //拟合椭圆
    Point2f led_on = Point2f(0,0);          //灯条上点坐标
    Point2f led_down = Point2f(0,0);    //灯条下点坐标
};


struct CarData{
    float pitch = 0;
    float yaw = 0;
    float ShootSpeed = 16;
    bool IsBuffMode = false;
    double BeginToNowTime = 0;
};

enum TuoluoStatus{
    STILL,              //静止
    SWITCH,        //切换
    RUN               //同块运动
};

enum TuoluoRunDirection{
    LEFT,
    RIGHT
};

struct TuoluoData{
    bool isTuoluo = false;                                          //是否为陀螺
    float R;                                                                      //半径
    float angle;                                                            //当前角度
    Point2f center;
    TuoluoStatus status;                                         //陀螺当前的状态
    TuoluoRunDirection runDirection;            //转动方向
    float spinSpeed;                                                //角速度
};


//得到两点距离
inline double GetDistance(Point2f a,Point2f b){
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}

//得到空间两点距离
inline double GetDistance(Point3f a,Point3f b){
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2)+pow(a.z - b.z,2));
}

//得到装甲中心
inline Point2f getArmorCenter(Point2f p[4]){
    return Point2f((p[0].x+p[1].x+p[2].x+p[3].x)/4,(p[0].y+p[1].y+p[2].y+p[3].y)/4);
}

//得到装甲倾斜角度
inline float getArmorAngle(Point2f p[4]){
    return((atan2(p[0].y - p[1].y,p[1].x - p[0].x)+atan2(p[3].y - p[2].y,p[2].x - p[3].x))/2);
}






#endif //FINDARMOR_TUOLUO_VARIABLES_H
