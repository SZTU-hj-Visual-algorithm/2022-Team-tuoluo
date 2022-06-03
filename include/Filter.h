/*********
  *Function List:
     1.EKF   构造函数包含有参构造和无参构造
     2.Prediction   传入状态矩阵,进行预测部分计算
     3.GetPrediction    包含有参和无参重载,无参代表直接使用类内状态向量和状态矩阵相乘,有参代表与传入状态矩阵相乘
     4.set_x    状态向量初始化
     5.update   状态更新
**********************************************************************************/
#ifndef FILTER_H
#define FILTER_H

#include<iostream>
#include<Eigen/Dense>
#include<stdio.h>
using namespace std;

//第二版一阶卡尔曼预测,角度版
class EKF{
private:
    double pitch;                                       //测量pitch角度,绕x轴左右转
    double yaw;                                       //测量yaw角度，绕y轴上下转

public:

    Eigen::VectorXd x_;                         //状态向量[锁定目标绝对pitch,锁定目标绝对yaw,v_pitch,v_yaw]，锁定目标时的pith和yaw，后面两个是速度


    EKF(Eigen::MatrixXd P_in , Eigen::MatrixXd Q_in, Eigen::MatrixXd H_in, Eigen::MatrixXd R_in);
    void Prediction(Eigen::MatrixXd _F);
    Eigen::VectorXd GetPrediction();
    Eigen::VectorXd GetPrediction(Eigen::MatrixXd _F);

        Eigen::MatrixXd F;                           //状态转移矩阵

        Eigen::MatrixXd P;                          //状态协方差矩阵
        Eigen::MatrixXd Q;                          //过程噪声

        Eigen::MatrixXd H;                          //测量矩阵

        Eigen::MatrixXd R;                          //测量噪声矩阵
        Eigen::MatrixXd K;                          //卡尔曼增益

        bool is_set_x = false;                     //判断是否赋初值

    EKF();                                          //创建，默认构造：P，Q，H，R
    void set_x(Eigen::VectorXd x);                                   //赋初值
    void set_x(Eigen::VectorXd x,Eigen::MatrixXd _F);                //赋初值，多个A
     
    void update(Eigen::VectorXd z,Eigen::MatrixXd _F);             //更新
    Eigen::VectorXd get_x();                                  //返回状态向量，更新后的x+

//    void initialize();                                    //初始化，没了

};

#endif // FILTER_H
