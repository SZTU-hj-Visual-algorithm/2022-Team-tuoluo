/*********************************************************************************
  *Description: 卡尔曼滤波工具类
  *Function List:
     1.EKF   构造函数包含有参构造和无参构造
     2.Prediction   传入状态矩阵,进行预测部分计算
     3.GetPrediction    包含有参和无参重载,无参代表直接使用类内状态向量和状态矩阵相乘,有参代表与传入状态矩阵相乘
     4.set_x    状态向量初始化
     5.update   状态更新
**********************************************************************************/
#include"Filter.h"
#include<Eigen/Dense>

//一阶卡尔曼预测

EKF::EKF(){
    //状态协方差矩阵附初值,搭配绝对位置的移动预测

    Eigen::MatrixXd P_in = Eigen::MatrixXd(2,2);
    P_in <<
        1.0, 0.0,
        0.0, 1.0;
    P = P_in;

     //过程噪声矩阵附初值
    Eigen::MatrixXd Q_in(2,2);
    Q_in <<
        1.0, 0.0,
        0.0, 1.0;
    Q = Q_in;

      //测量矩阵附初值
    Eigen::MatrixXd H_in(1,2);
    H_in <<
        1.0, 0.0;
    H = H_in;

    //测量噪声矩阵附初值
    Eigen::MatrixXd R_in(1,1);
    R_in <<
        1.0;
    R = R_in;


}

/**
 * @brief EKF类初始化重载
 * @param P_in状态协方差矩阵   Q_in过程噪声矩阵  H_in测量矩阵    R_in测量噪声矩阵
 */
EKF::EKF(Eigen::MatrixXd P_in , Eigen::MatrixXd Q_in,Eigen::MatrixXd H_in,Eigen::MatrixXd R_in){
    P = P_in;
    Q = Q_in;
    H = H_in;
    R = R_in;
}

/**
 * @brief 给状态向量附初值
 * @param x 状态向量初值      _F状态转移矩阵
 */
void EKF::set_x(Eigen::VectorXd x,Eigen::MatrixXd _F){
    F = _F;
    x_ = x;
    is_set_x = true;
}

void EKF::set_x(Eigen::VectorXd x){
    x_ = x;
    is_set_x = true;
}

/**
 * @brief EKF类初始化重载
 * @param _F对应当前状态的状态转移矩阵
 */
void EKF::Prediction(Eigen::MatrixXd _F){
    F = _F;
    //得到预测值
    x_ = F * x_;
    P = F * P * F.transpose() + Q;
}

/**
 * @brief EKF 返回相乘量
 * @param _F对应当前状态的状态转移矩阵
 * @return 状态向量
 */
Eigen::VectorXd EKF::GetPrediction(Eigen::MatrixXd _F){
    return _F * x_;
}

/**
 * @brief EKF::GetPrediction
 */
Eigen::VectorXd EKF::GetPrediction(){
    return F * x_;
}

/**
 * @brief 返回状态向量，获得预测值
 * @return 状态向量
 */
Eigen::VectorXd EKF::get_x(){
    return x_;
}

//更新状态
void EKF::update(Eigen::VectorXd z,Eigen::MatrixXd _F){
    F = _F;
    Eigen::MatrixXd y = z - H * x_;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x_ = x_ + (K * y);
    int size = x_.size();

    Eigen::MatrixXd I = Eigen::MatrixXd(size,size);

    if(size == 2){   
        I <<
            1, 0,
            0, 1;   
    }
    if (size == 4) {
        Eigen::MatrixXd I(4, 4);
        I <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    }
    if(size == 6){
        Eigen::MatrixXd I(6,6);
        I << 
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    }
    P = (I - K * H) * P;
}

