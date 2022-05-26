#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
//#include <opencv2/video/tracking.hpp>

#include "kal_filter.hpp"
#include "robot_state.h"

using namespace cv;

class KAL:public robot_state
{
private:
	double depth;
	
	Mat F_MAT;
	Eigen::Matrix3d F;
	
	Mat C_MAT;
	Eigen::Matrix<double, 1, 5>C;
	
	int measure_a;
	
	int state_a;
	
	float t = -1;
	
//	int stop_predict = 0;
	
	double last_aim_yaw = 0;
	double last_aim_pitch = 0;
	
	float shoot_delay = 0.40;
	
	double filter = 0.8;

public:
	KAL() = default;
	
	void reset();
	kal_filter init();
	float keep_pi(float angle);
	Eigen::Vector3d pnp_get_pc(const cv::Point2f p[4], const double& w, const double& h);
	
	bool predict(RotatedRect& detection, kal_filter & kf, double t);
	
	inline Eigen::Vector3d pu_to_pc(Eigen::Vector3d& pu)
	{
		return F.inverse()*(pu * depth) ;//transpose�����ת��,inverse��������
	}
	
	inline Eigen::Vector3d pc_to_pu(Eigen::Vector3d& pc)
	{
		return F * pc / depth;
	}
	
	Mat _src;
	int type;
//	int send_flag = 0;
	
	struct information {
		float yaw = 0.0;
		float pitch = 0.0;
	};
	
	information send;
	
//	float ab_pitch = 0.0;
//	float ab_yaw = 0.0;
//	float ab_roll = 0.0;
//	float SPEED = 25.0;
	
	
};
