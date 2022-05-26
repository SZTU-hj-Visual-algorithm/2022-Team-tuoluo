//
// Created by liyankuan on 2022/4/17.
//

#ifndef DEMO_ROBOT_STATE_H
#define DEMO_ROBOT_STATE_H
#include <opencv2/opencv.hpp>
#include "tuoluo_variables.h"

struct Final_Armor{
    float tx = 0;
    float ty = 0;
    float tz = 0;
    float pitch = 0;
    float yaw = 0;
    float bar_angle_abs;   //灯条角度（绝对值）
    float armor_angle;     //装甲角度(绝对值)
    bool IsSmall = true;
    bool IsTuoluo = false;          //是否为陀螺状态
    double get_width = 0;               //对应陀螺宽度
    double distance = 0;
    led leds[2];
    int status;
    cv::Point2f point[4];
    bool IsShooting = true;
};


enum EnemyColor { RED = 0, BLUE = 1 };
class robot_state
{
public:
	float ab_pitch = 0.0;
	float ab_yaw = 0.0;
	float ab_roll = 0.0;
	float SPEED = 25.0;
    Final_Armor BestArmor;
	int enemy_color;

};

#endif //DEMO_ROBOT_STATE_H
