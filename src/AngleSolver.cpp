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
#include "AngleSolver.h"
#include "Filter.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "robot_state.h"
#include "tuoluo_detect.h"

#define PI 3.14
#define SHOOT_DELAY_TIME 120                    //发弹延迟
#define MIN_FORCE_DISTANCE_CHA  10000           //符合使用的最小距离差
#define USING_ANGLE_FILTER                      //使用角度滤波
#define RECORD_RUN_TIMES 1                      //记录移动数组容量
//#define USING_POISITION_FILTER                  //使用位置滤波
//#define USING_POISITION_A


/* run为一定时间t内云台转动角度量 */

AngleSolver AS;


//定义卡尔曼类型
EKF KF_tuoluo;

/********************击打置信级数***************************/

#define MIN_SHOOT_TRUST_LEVEL 3         //最小满足击打的置信等级

int ForceShootTrustlevel = 0;
float RunSpeed[MIN_SHOOT_TRUST_LEVEL];
/********************击打置信级数***************************/

/********************第一版(角度)***************************/
//定义运动绝对角度
float realCarPitch = 0;
float realCarYaw = 0;
//用于缓冲计算
float SendPitch = 0;                //记录上一帧pitch发送角度量
float SendYaw = 0;                //记录上一帧yaw发送角度量
//预测计算保留量
CarData old_carDatas;                //上次收数
float old_Pitch= 0;            //上级Pitch相对角度
float old_Yaw = 0;              //上级Yaw相对角度

CarData buffer_old_carDatas ;     //缓冲保留计算量
bool IsHaveBuffer = false;

//保留最新几帧的目标移动速度
float PitchRunSpeed[RECORD_RUN_TIMES];
float YawRunSpeed[RECORD_RUN_TIMES];
int RecordRunTimes = 0;

/*****************************************************/

/********************第二版(位置)***************************/
/*****************************************************/
//陀螺连续保留量
ShootTuoluo shootTuoluo;
bool isFirstShootTuoluo = true;     //是否当前情况为首次发现陀螺目标
float old_angle = 0;                            //上一次陀螺角度
float old_SpinSpeed = 0;                 //上一次陀螺角速度
bool isSetKF_tuoluo = false;         //是否设置过陀螺卡尔曼矩阵
bool isSetKF_SecOrder = false;         //是否设置过陀螺卡尔曼矩阵
bool isSetAngleKF = false;
//记录前后时间
double  old_CarTime = 0;


AngleSolver::AngleSolver() {
    //类成员初始化
    fBigArmorWidth = 22.5;
    fBigArmorHeight = 8.85;
    fSmallArmorWidth = 13.5;
    fSmallArmorHeight = 8.55;        //原5.5cm

    //坐标系转换
    ptz_camera_y = 2;                    //相机与云台垂直方向距离差
    ptz_camera_z = 10.5;                  //相机与云台轴向方向距离差
    ptz_camera_x = 2;                    //相机与云台水平方向距离差

    ptz_camera_pitch = 0 * PI / 180;            //对应绕x旋转角度,弧度，角度乘0.017453
    ptz_camera_yaw = 0 * PI / 180;               //对应绕y旋转角度，弧度
    ptz_camera_roll = 0 * PI / 180;                 //对应绕z旋转角度，弧度

    //陀螺滤波初始化,仅需执行一次
    if (!isSetKF_tuoluo) {
        isSetKF_tuoluo = true;
        //状态协方差矩阵附初值

        Eigen::MatrixXd P_in = Eigen::MatrixXd(2, 2);
        P_in << 1.0, 0.0,
            0.0, 1.0;
        KF_tuoluo.P = P_in;

        //过程噪声矩阵附初值
        Eigen::MatrixXd Q_in(2, 2);
        Q_in << 1.0, 0.0,
            0.0, 1e-1;
        KF_tuoluo.Q = Q_in;

        //测量矩阵附初值
        Eigen::MatrixXd H_in(1, 2);
        H_in << 1.0, 0.0;
        KF_tuoluo.H = H_in;

        //测量噪声矩阵附初值
        Eigen::MatrixXd R_in(1, 1);
        R_in << 8500;
        KF_tuoluo.R = R_in;

        Eigen::MatrixXd F_in(2, 2);
        F_in <<
            1.0, 1.0,
            0.0, 1.0;
        KF_tuoluo.F = F_in;
    }
}

void GetPoint2D(Final_Armor& BestArmor, std::vector<cv::Point2f>& point2D) {

    cv::Point2f lu, ld, ru, rd;        //right_down right_up left_up left_down

    lu = BestArmor.point[0];
    ld = BestArmor.point[3];
    ru = BestArmor.point[1];
    rd = BestArmor.point[2];

    //     cout<<"lu:"<<lu<<endl;
    point2D.clear();///先清空再存入
    point2D.push_back(lu);
    point2D.push_back(ru);
    point2D.push_back(rd);
    point2D.push_back(ld);
}

//矩形转换为3d坐标                                                                                                                                                                                             3
void GetPoint3D(Final_Armor& BestArmor, std::vector<cv::Point3f>& point3D)
{
    float fHalfX = 0;
    float fHalfY = 0;
    if (BestArmor.IsSmall)
    {
        //        cout<<"小"<<endl;
        fHalfX = AS.fSmallArmorWidth / 2.0;
        fHalfY = AS.fSmallArmorHeight / 2.0;
    }
    else {
        //         cout<<"大"<<endl;
        fHalfX = AS.fBigArmorWidth / 2.0;
        fHalfY = AS.fBigArmorHeight / 2.0;
    }
    point3D.push_back(cv::Point3f(-fHalfX, -fHalfY, 0.0));
    point3D.push_back(cv::Point3f(fHalfX, -fHalfY, 0.0));
    point3D.push_back(cv::Point3f(fHalfX, fHalfY, 0.0));
    point3D.push_back(cv::Point3f(-fHalfX, fHalfY, 0.0));

}


//pnp转换,得到目标坐标
void CountAngleXY(const std::vector<cv::Point2f>& point2D, const std::vector<cv::Point3f>& point3D, Final_Armor& BestArmor, Camera0 camera) {
    cv::Mat rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvecs = cv::Mat::zeros(3, 1, CV_64FC1);


    cv::solvePnP(point3D, point2D, camera.caremaMatrix, camera.distCoeffs, rvecs, tvecs);

    //     cout<<"平移向量："<<tvecs<<endl;
    double tx = tvecs.ptr<double>(0)[0];
    double ty = tvecs.ptr<double>(0)[1];
    double tz = tvecs.ptr<double>(0)[2];

    BestArmor.tx = tx;
    BestArmor.ty = ty;
    BestArmor.tz = tz;

    BestArmor.pitch = atan2(BestArmor.ty, BestArmor.tz) * 180 / PI;
    BestArmor.yaw = atan2(BestArmor.tx, BestArmor.tz) * 180 / PI;
}

/**
 * @brief AngleSolver::ShootRevice      相机与枪口坐标调整
 * @param tx
 * @param ty
 * @param tz
 * 旋转时以逆时针为正                pitch转换为绝对角度计算抬升
 */
void ShootAdjust(float& tx, float& ty, float& tz, float Carpitch, float Caryaw) {
    //角度转弧度
    Carpitch *= PI / 180;
    Caryaw *= PI / 180;

    //变回原来的点
    Carpitch *= -1;
    Caryaw *= -1;

    //绕roll轴旋转，即为绕z轴旋转
    Eigen::MatrixXd r_Roll(3, 3);
    r_Roll << 1, 0, 0,
        0, cos(AS.ptz_camera_roll), sin(AS.ptz_camera_roll),
        0, -sin(AS.ptz_camera_roll), cos(AS.ptz_camera_roll);

    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3, 3);
    r_Pitch << cos(AS.ptz_camera_pitch), 0, -sin(AS.ptz_camera_pitch),
        0, 1, 0,
        sin(AS.ptz_camera_pitch), 0, cos(AS.ptz_camera_pitch);

    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3, 3);
    r_Yaw << cos(AS.ptz_camera_yaw), sin(AS.ptz_camera_yaw), 0,
        -sin(AS.ptz_camera_yaw), cos(AS.ptz_camera_yaw), 0,
        0, 0, 1;

    Eigen::VectorXd original(3, 1);          //按z，x，y传入，即变化对应到左手坐标系
    original << tz, tx, ty;


    //平移变换,先旋转再平移        
    Eigen::VectorXd translation(3, 1);
    translation << AS.ptz_camera_z, AS.ptz_camera_x, AS.ptz_camera_y;
    original = original - translation;                               //////////平移步骤，相机到枪口的平移


    Eigen::VectorXd change(3, 1);
    //旋转变换，连续相乘
    change = r_Roll * original;
    change = r_Pitch * change;
    change = r_Yaw * change;
    //以上部分的偏移参数调节


    //去掉车云台旋转相对影响,坐标系转换到相对初始位的绝对坐标，，，，相对处理
    //pitch转换
    Eigen::MatrixXd r_pitch_car(3, 3);
    r_pitch_car << cos(Carpitch), 0, -sin(Carpitch),
        0, 1, 0,
        sin(Carpitch), 0, cos(Carpitch);
    //    change = r_pitch_car*change;
        //yaw转换
    Eigen::MatrixXd r_yaw_car(3, 3);
    r_yaw_car << cos(Caryaw), sin(Caryaw), 0,
        -sin(Caryaw), cos(Caryaw), 0,
        0, 0, 1;

    //下面两步是旋转变换
    change = r_pitch_car * change;
    change = r_yaw_car * change;


    //重新更新，枪口到中心
    tx = change(1);
    ty = change(2);
    tz = change(0);
}


/**
 * @brief AngleSolver::ComputeShoot         计算射击缓冲  及时间
 * @param tx
 * @param ty
 * @param tz
 * @return
 */
Angle_t ComputeShootTime(float tx, float ty, float distance, struct CarData CarDatas) {
    //g表示重力加速度，LevelDistance表示水平距离(sqrt(pow(tz,2)+pow(tx,2)),HeightDistance为垂直高度ty，v为射速，tan_angle为所要求俯仰角的正切值
    //计算公式: -0.5g*pow(LevelDistance,2)/pow(V,2)*pow(tan_angle,2) + tan_angle*LevelDistance - 0.5g*pow(LevelDistance,2)/pow(V,2) - HeightDistance   
    //计算时间使用t = LevelDistance/(v*cos_angle)


    //单位转换
    tx /= 1000.0;
    ty /= 1000.0;
    float tz = distance / 1000.0;
    //    cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<distance<<endl;
    float speed = CarDatas.ShootSpeed;
    //    speed=20;
    double a = -0.5 * 9.8 * (pow(tz, 2) + pow(tx, 2));
    double b = sqrt((pow(tz, 2) + pow(tx, 2))) * pow(speed, 2);
    double c = -0.5 * 9.8 * (pow(tz, 2) + pow(tx, 2)) - pow(speed, 2) * ty;
    //判别式
    double Discriminant = pow(a, 2) + pow(b, 2) - 4 * a * c;
    //    cout<<"判别式:"<<Discriminant<<"   a:"<<a<<"   b:"<<b<<"   c:"<<c<<endl;
    Angle_t ShootBuff = { 0,0,0,0 };
    if (Discriminant < 0)return ShootBuff;
    double angle_tan_1 = atan((-b + sqrt(Discriminant)) / (2 * a)) * 180 / PI;
    double angle_tan_2 = atan((-b - sqrt(Discriminant)) / (2 * a)) * 180 / PI;
    //    double angle_tan = ty/b;
    //    double real_angle = fabs(angle_tan - angle_tan_1)<fabs(angle_tan - angle_tan_2)?angle_tan_1:angle_tan_2;
        //角度取舍,并转换为相对角度
    cout << "计算所得打击角度1:" << angle_tan_1 << "  计算所得打击角度2:" << angle_tan_2 << endl;
    if (fabs(angle_tan_1) <= fabs(angle_tan_2) && fabs(angle_tan_1) < 45) {
        ShootBuff.pitch = angle_tan_1 - CarDatas.pitch;
    }
    else if (fabs(angle_tan_2) < 45) {
        ShootBuff.pitch = angle_tan_2 - CarDatas.pitch;
    }
    else {      //都不符合要求
        cout << "计算解不符合实际" << endl;
        return ShootBuff;
    }

    //    real_angle = atan(real_angle)*180/PI;
    //    ShootBuff.pitch = real_angle;
    ShootBuff.yaw = atan2(tx, tz) * 180 / CV_PI;
    cout << "缓冲计算tx:" << tx << "  ty:" << ty << " tz:" << tz << "yaw" << ShootBuff.yaw << "最小角度" << atan2(ty, tz) * 180 / PI << endl;
    ShootBuff.t = tz / (speed * cos(ShootBuff.pitch * PI / 180)) * 1000;       ////////////
    cout << "击打时间:" << ShootBuff.t << endl;
    return ShootBuff;
}

/**
 * @brief AngleSolver::ComShootPosition     陀螺击打点计算
 * @param BestArmor                                             当前目标
 * @param tuoluoData                                            陀螺数据
 * @param nowTime                                                          当前的所得图片对应时间
 * @param oldTime                                                          上一图片对应所得时间
 */
void ComTuoluoShootPosition(Final_Armor& BestArmor, TuoluoData tuoluoData, double nowTime, double oldTime, CarData carDatas) {
    //    //利用角度解算中ComputeShootTime函数计算击打时间
    //    tuoluoShootTime = AngleSolver::ComputeShootTime(BestArmor.tx,BestArmor.ty,BestArmor.tz);
    //    if(tuoluoShootTime.t == 0) return;              //击打时间计算无解
        //滤波过滤角速度
    if (!KF_tuoluo.is_set_x) {            //第一次得到角度量
        //设置状态向量和状态转移矩阵
        Eigen::MatrixXd F(2, 2);
        F <<
            1, 0,
            1, 1;
        //设置状态向量    [角速度,角速度变换量],角速度变换量默认初值0
        Eigen::VectorXd x(2, 1);
        x << tuoluoData.spinSpeed / (nowTime - oldTime), 0;       // 角加速度？
        //卡尔曼滤波赋初值
        KF_tuoluo.set_x(x, F);
    }
    else {
        //连续滤波
        Eigen::MatrixXd F(2, 2);
        F << 1, 0,
            (nowTime - oldTime), 1;
        KF_tuoluo.Prediction(F);
        Eigen::VectorXd z(1, 1);
        z << tuoluoData.spinSpeed / (nowTime - oldTime);
        KF_tuoluo.update(z, F);
    }

    //计算击打时间,当前未添加迭代,原理上应添加迭代,因为当前所计算的位置并不是最终预测位置
    Angle_t tuoluoShootTime = ComputeShootTime(BestArmor.tx, BestArmor.ty, BestArmor.tz, carDatas);
    if (tuoluoShootTime.t == 0) return;             //计算无解
    //得到角度预测量      弹道时间+本帧运行时间+发弹延迟
    double bufferAngle = KF_tuoluo.x_(0) * (tuoluoShootTime.t + Recive.getClock() - nowTime + SHOOT_DELAY_TIME);

    //计算击打位置,选择击打模块
    if (fabs(bufferAngle) >= 360) {
        bufferAngle = bufferAngle - ((int)bufferAngle / 360) * 360;
    }
    float goalAngle = tuoluoData.angle + bufferAngle;
    //防止所得角度绝对值超过360度
    if (goalAngle >= 360 || goalAngle <= -360) {
        goalAngle = goalAngle - ((int)bufferAngle / 360) * 360;
    }
    //防止所得角度绝对值大于180
    if (goalAngle >= 180) {
        goalAngle = -360 + goalAngle;
    }
    else if (goalAngle <= -180) {
        goalAngle = 360 + goalAngle;
    }

    float minAngle = goalAngle;
    //如果计算到的角度绝对值小于20度也就是大概在中心就击打
    if (fabs(minAngle) > 20) {
        //根据当前目标角度往右计算每块装甲对应角度,也就是俯视逆时针看
        for (int i = 0; i < 3; i++) {
            float newAngle = goalAngle + 90;
            //防止所得角度大于180
            if (newAngle > 180) {
                newAngle = -180 + (newAngle - 180);
            }
            if (fabs(newAngle) > fabs(minAngle)) {
                minAngle = newAngle;
            }
            if (fabs(minAngle) < 20)break;
        }
    }

    //根据角度,陀螺圆心及陀螺半径计算预测点
    if (minAngle > 50 || minAngle < -50) {
        cout << "当前陀螺预测角度计算有误!!!" << endl;
        cout << "当前所计算角速度:" << KF_tuoluo.x_(0) << "  未滤波的原速度:" << tuoluoData.spinSpeed << endl;
        cout << "当前所计算的击打角度:" << minAngle << endl;
        return;
    }
    //计算所得结果符合一定范围内
    //无论左右均可,前提x坐标系是右大左小,与像素坐标系相同
    BestArmor.tz = tuoluoData.center.y - tuoluoData.R * cos(minAngle * PI / 180);
    BestArmor.tx = tuoluoData.center.x + tuoluoData.R * sin(minAngle * PI / 180);

    //计算击打抬升量
    Angle_t tuoluoShoot = ComputeShootTime(BestArmor.tx, BestArmor.ty, BestArmor.tz, carDatas);
    BestArmor.pitch = tuoluoShoot.pitch;
    BestArmor.yaw = tuoluoShoot.yaw;
}


/**
 * @brief AngleSolver::GetArmorAngle        角度解算
 * @param Src
 * @param _camera  摄像头信息
 * @param CarDatas  内角度现应为弧度
 * @remarks 计算绝对的打击角度时,ty坐标需使用绝对坐标,而tx和tz坐标需使用相对坐标
 */
void GetArmorAngle(Mat Src, Final_Armor& BestArmor, Camera0 _camera, CarData CarDatas) {
    std::vector<cv::Point2f>point2D;
    std::vector<cv::Point3f>point3D;

    GetPoint2D(BestArmor, point2D);                                                           //矩形转换为2d坐标
    GetPoint3D(BestArmor, point3D);                                                          //矩形转换为3d坐标
    CountAngleXY(point2D, point3D, BestArmor, _camera);                                             //解决pnp问题

    //相机坐标与云台初始枪口坐标转换,坐标系做云台当前角度反向旋转得到绝对坐标
//    return;
    Point3f RelativePoisition = Point3f(BestArmor.tx, BestArmor.ty, BestArmor.tz);              //保留相对坐标


    ShootAdjust(BestArmor.tx, BestArmor.ty, BestArmor.tz, CarDatas.pitch, CarDatas.yaw);     //转换为绝对坐标，枪口对装甲板

//    return;
    RelativePoisition.y = BestArmor.ty;                       //ty坐标（上下）使用绝对坐标

//    return;
    //判断陀螺
    TuoluoData tuoluoData = shootTuoluo.getTuoluoData(Src, BestArmor);
    if (tuoluoData.isTuoluo) {
        //陀螺击打,
        if (tuoluoData.runDirection == LEFT) {                        //标志左右方向
            circle(Src, Point(20, 20), 5, Scalar(255, 0, 0), -1, 8);
        }
        else {
            circle(Src, Point(20, 20), 5, Scalar(0, 0, 255), -1, 8);
        }
        ComTuoluoShootPosition(BestArmor, tuoluoData, CarDatas.BeginToNowTime, old_CarTime, CarDatas);
    }

    //根据绝对坐标得到当前相对的应旋转的角度
//    AbsToRelative(BestArmor.Armor.tx,BestArmor.Armor.ty,BestArmor.Armor.tz,CarDatas.pitch,CarDatas.yaw,BestArmor.Armor.pitch,BestArmor.Armor.yaw);

//    BestArmor.Armor.pitch += 0.8;
    if (BestArmor.status != buffering) {
        //当前不处于缓冲状态就更新记录时间
        //缓冲状态应不参与滤波计算,根据最后的滤波所得速度计算缓冲时间内绝对坐标,由绝对坐标计算相对角度,会产生丢失目标僵直状态
        old_CarTime = CarDatas.BeginToNowTime;
    }
}


/**
 * @brief AngleSolver::AbsToRelative        由云台当前角度和绝对坐标得到相对角度,然后通过参数（取地址）返回
 * @param tx
 * @param ty
 * @param tz
 * @param Carpitch  车云台pitch角度
 * @param Caryaw    车云台yaw角度
 * @param pitch         返回识别相对pitch角度
 * @param yaw           返回识别相对yaw角度
 * @remark 车pitch和yaw角度均为0~180和0~-180
 */
void AbsToRelative(const float tx, const float ty, const float tz, float Carpitch, float Caryaw, float& pitch, float& yaw) {
    float distance = sqrt(pow(tx, 2) + pow(ty, 2) + pow(tz, 2));           //距离
    float absPitch = asin(fabs(ty) / distance) * 180 / PI;                                       //pitch轴角度绝对值

    //确定pitch角度
    if (tz >= 0) {
        //前
        if (ty >= 0) {
            //上方
            pitch = absPitch;
        }
        else {
            //下方
            pitch = -absPitch;
        }
    }
    else {
        //后
        if (ty >= 0) {
            //上方
            pitch = 180 - absPitch;
        }
        else {
            //下方
            pitch = -180 + absPitch;
        }
    }

    //确定yaw角度
    if (tx >= 0) {
        //原点右侧
        if (tz == 0 && tx > 0)       //特殊求tan分母为0的情况
            yaw = 90;

        if (tz > 0) {                       //一般情况
            yaw = atan(tx / tz) * 180 / PI;
        }
        else {
            yaw = 180 - atan(tx / (-tz)) * 180 / PI;
        }

    }
    else {
        //原点左侧
        if (tz == 0)       //特殊求tan分母为0的情况
            yaw = -90;

        if (tz > 0) {                       //一般情况
            yaw = atan(tx / tz) * 180 / PI;
        }
        else {
            yaw = -180 - atan(tx / (-tz)) * 180 / PI;
        }

    }

    //    Carpitch = Carpitch*180/PI;
    //    Caryaw = Caryaw*180/PI;
    Caryaw = Caryaw - (int)Caryaw / 360 * 360;
    if (Caryaw > 180)
        Caryaw -= 360;
    if (Caryaw < -180)
        Caryaw += 360;
    //由绝对角度得到相对角度
    pitch -= Carpitch;
    yaw -= Caryaw;

}


