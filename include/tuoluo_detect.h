/*
 *Description: 陀螺判断
  * Function List:
     1.getTuoluoData    陀螺检测接口,传入当前处理图片和识别信息,内部自动连续处理,反馈识别结果
     2.firstSetTuoluo     第一次发现目标,将各项积累值清零
     3.continueSetTuoluo    第一种连续判断目标运动状态函数
     3.SecContinueSetTuoluo     第二种连续检测方案,添加计算最近几帧的目标角度方差作为判断依据
     4. drawImage   显示判断为陀螺状态时目标图像
  * Others:利用单目测距和三角形相似原理得到目标运动规律,估计目标当前是否处于陀螺状态,并给出具体击打所需信息
 */






//
// Created by TanSiwei on 2022/5/14.
//

#ifndef FINDARMOR_TUOLUO_SHOOT_H
#define FINDARMOR_TUOLUO_SHOOT_H

#include<tuoluo_variables.h>
#include<ArmorDetector.hpp>



class ShootTuoluo{
private:
    double tz_armor_width_ ;  //装甲板在标准距离下，对应的宽度，用于后续陀螺计算
    double tz_normal ;
public:
    TuoluoData getTuoluoData(Mat Src,Final_Armor BestArmor);
    ShootTuoluo();

private:
    double getAngle(Final_Armor BestArmor);
    void firstSetTuoluo(double angle,Final_Armor armor);
    //第二种陀螺检测方案
    bool ContinueSetTuoluo(Mat Src,double & angle,Final_Armor BestArmor,double & R,Point2f & center,TuoluoStatus & status,TuoluoRunDirection & direction,float & spinSpeed);
    void drawImage(Mat Src,double angle,Point2f object,double R,float armorWidth);
};

struct Angle_tz{
    double angle = 90;
    float tz = 0;
    float r = 0;
};




















#endif //FINDARMOR_TUOLUO_SHOOT_H
