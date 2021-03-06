#include "KAl.h"


#define draw_circle
#define draw_height
#define show_circle


kal_filter KAL::init()
{
	//cv::FileStorage fin("/home/hsy/����/zhenhe/linux_project6/other/calib_no_4_12801(1).yml", cv::FileStorage::READ);
	//fin["F"] >> F_MAT;//�ڲξ���
	//fin["C"] >> C_MAT;//�������
	F_MAT=(Mat_<double>(3, 3) << 1583.14676, 0.000000000000, 633.90210, 0.000000000000, 1582.31678, 528.53893, 0.000000000000, 0.000000000000, 1.000000000000);
	C_MAT=(Mat_<double>(1, 5) << -0.08767, 0.19792, 0.00021, 0.00068, 0.00000);
	
	cv2eigen(F_MAT, F);
	cv2eigen(C_MAT, C);
	
	kal_filter kf;//��ʼ��kalman�˲�
	
	return kf;
}

void KAL::reset()
{
	t = -1;
	//stop_predict = 0;
//	send_flag = 0;
}

bool KAL::predict(RotatedRect &detection, kal_filter& kf, double time)
{
	if (detection.size.empty())
	{
		reset();
		send.pitch = 0.0;
		send.yaw = 0.0;
		printf("no aim!!");
		return false;
	}
	//����ͼ���ϼ�����Ŀ�����ڵ����������˲��õ��������ֵ--------------------------------
	//Point bp = detection.center;
//	if (bp.x - last_center.x < 5 && bp.y - last_center.y < 5)
//	{
//		stop_predict++;
//	}
//	else
//	{
//		stop_predict = 0;
//	}
//	last_center = bp;
	//���Ŀ����������------------------------------------------------------
	double w, h;
	if (type == 1)
	{
		w = 0.130;
		h = 0.055;
	}
	else
	{
		w = 0.225;
		h = 0.055;
	}
	
	cv::Point2f pts[4];
	detection.points(pts);
	std::sort(pts, pts + 4, [](const cv::Point2f& p1, const cv::Point2f& p2) { return p1.x < p2.x; });
	Eigen::Vector3d m_pc = pnp_get_pc(pts, w, h);
	double depth = m_pc(2, 0);
	
	double ra_yaw = atan2(m_pc(0,0),m_pc(2,0)) / CV_PI*180.0;
	double ra_pitch = atan2(m_pc(1,0),m_pc(2,0)) / CV_PI*180.0;
	
//	double aim_yaw = -ab_yaw+ ra_yaw ;
//	double aim_pitch = -ab_pitch + ra_pitch ; //以度为单位
	
//	double aim_x = depth * tan(aim_yaw/180.0 * PI);
//	double aim_y = depth * tan(aim_pitch/180.0 * PI);//解出目标在绝对世界坐标系中的xy坐标
	Mat eular = (Mat_<float>(3,1)<<ab_pitch/180.0*CV_PI,ab_yaw/180.0*CV_PI,ab_roll/180.0*CV_PI);
	Mat rotated_mat;
	Eigen::Matrix<double,3,3> rotated_matrix;
    	cv::Rodrigues(eular,rotated_mat);
	cv2eigen(rotated_mat,rotated_matrix);
	Eigen::Vector3d m_pd = rotated_matrix.inverse()*m_pc;
	
	if (t == -1)
	{
		t = time;
		send.yaw = ra_yaw;
		send.pitch = ra_pitch;
		kf.Xk_1[0] = m_pd(0,0);
		kf.Xk_1[3] = m_pd(1,0);
//		last_aim_pitch = ra_pitch - ab_pitch;
//		last_aim_yaw = ra_yaw - ab_yaw;
		return true;
	}
	
	
//	if (aim_yaw - last_aim_yaw < 5 && aim_pitch - last_aim_pitch < 5)
//	{
//		stop_predict++;
//	}
//	else
//	{
//		stop_predict = 0;
//	}
//
	if (last_aim_yaw ==0 && last_aim_pitch ==0)
	{
		last_aim_pitch = ra_pitch;
		last_aim_yaw = ra_yaw;
	}

	
	//-------------------------------------------------------------------------
	
	Eigen::Matrix<double, 2, 1> measured;
	measured << m_pd(0,0), m_pd(1,0);
	
	double delata_t = (time - t)/getTickFrequency();
	t = time;
	
	Eigen::Matrix<double, 6, 1> pred = kf.predict(delata_t,false);//������һʱ�̵ĺ������Ԥ�����һʱ�̵��������ֵ
	
	Eigen::Matrix<double, 6, 1> corrected = kf.correct(measured);//��������Э������󣬲�����������
	
	
	//-----------------------------------------------------------------------------------------
	
	
	
	
	//�õ�Ŀ����ά���꣬����ӵ����·���ʱ�䣬�������Ŀ�����ӵ����к�����ӳ�ʱ���ڻ��߹���x��yֵ----------
	
	
	double predict_time = m_pc.norm() / SPEED + shoot_delay;
	
	
	Eigen::Matrix<double, 6, 1> pre_xy = kf.predict(predict_time,true);
	
	//预测得到的目标绝对角度----------------------------------------------------------
	
//	double need_yaw;
//	double need_pitch;
	//-------------------------------------------------------------------------------
	
	
	//--����������ϵ�µ�Ԥ���λӳ�䵽�������ϵ��---------------------------------------
//	if (stop_predict >= 8)
//	{
//		need_yaw = corrected[0] + ab_yaw;
//		need_pitch = corrected[3] + ab_pitch;
//	}
//	else
//	{
//	need_yaw = atan2(pre_xy[0] , depth)/PI*180.0 + ab_yaw;
//	need_pitch = atan2(pre_xy[3] , depth)/PI*180.0 + ab_pitch;
//	}
	
	
	Eigen::Vector3d pos3 = {pre_xy[0],pre_xy[3],m_pd(2,0)};
	printf("x_v:%lf\tx_a:%lf\ny_v:%lf\ty_a:%lf\n",pre_xy[1],pre_xy[2],pre_xy[4],pre_xy[5]);
	if (pre_xy[1]< 0.026 && pre_xy[2]< 0.016 && pre_xy[4]< 0.006 && pre_xy[5]< 0.0032)
	{
		send.yaw = (1.0-filter)*ra_yaw + filter*last_aim_yaw;
		send.pitch = (1.0-filter)*ra_pitch + filter*last_aim_pitch;
		last_aim_pitch = ra_pitch;
		last_aim_yaw = ra_yaw;
		return true;
	}
	last_aim_pitch = ra_pitch;
	last_aim_yaw = ra_yaw;
	pos3 = rotated_matrix*pos3;
	pos3 = {pos3[0],pos3[1],depth};



#ifdef draw_circle
	//������----------------------------------------------------------------------
	Eigen::Vector3d pos2 = pc_to_pu(pos3);
	//circle(_src, bp, 7, Scalar(0, 0, 255), 3);
	//由角度预测得到的目标点在图像上的映射位置
	circle(_src, Point((int)pos2[0],(int)pos2[1]), 7, Scalar(255, 0, 0), 3);
	//---------------------------------------------------------------------
#endif
	
	
	//-----------------------------------------------------------------------------------
	
	if (SPEED == 0.0)
	{
		SPEED = 25.0;
	}
	//-----̧ǹ����----------------------------------------------------------------------
	//printf("pos3.No3:%f\t%f\n",pos3[2]);
	double del_ta = pow(SPEED, 4) + 2 * 9.8 * pos3(1, 0) * SPEED * SPEED - 9.8 * 9.8 * pos3(2, 0) * pos3(2, 0);
	
	double t_2 = (9.8 * pos3(1, 0) + SPEED * SPEED - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);
	
	double height = 0.5 * 9.8 * t_2;
	//printf("height:%f\n",height);
	//------------------------------------------------------------------------------------
#ifdef draw_height
	Eigen::Vector3d ap_pre{ pos3(0,0) , pos3(1,0) - height , pos3(2,0) };
	
	Eigen::Vector3d pu_pre = pc_to_pu(ap_pre);

	circle(_src, Point((int)pu_pre(0, 0), (int)pu_pre(1, 0)), 7, Scalar(255, 255, 0), 3);
#endif
#ifdef show_circle
	imshow("src_kal",_src);
#endif
	
	
	//-----����ǶȺ���Ҫ���͸���ص�����------------------------------------------------
	//�������нǶȶ���תΪ�Ƕ���
	send.yaw = atan2(pos3(0, 0), pos3(2, 0))/CV_PI * 180.0;//atan2�ĽǶȷ�Χ��-180~180���պ����нǶȶ��и���,������
	send.pitch = atan2(pos3(1, 0) /*+ 0.075*/ - height , pos3(2, 0))/CV_PI * 180.0;
	//-----------------------------------------------------------------------------------


	return true;
	
}

Eigen::Vector3d KAL::pnp_get_pc(const cv::Point2f p[4], const double& w, const double& h)
{
	cv::Point2f lu, ld, ru, rd;
	std::vector<cv::Point3d> ps = {
			{-w / 2 , -h / 2, 0.},
			{w / 2 , -h / 2, 0.},
			{w / 2 , h / 2, 0.},
			{-w / 2 , h / 2, 0.}
	};
	if (p[0].y < p[1].y) {
		lu = p[0];   ////����
		ld = p[1];	////����
	}
	else {
		lu = p[1];
		ld = p[0];
	}
	if (p[2].y < p[3].y) {
		ru = p[2];   ////����
		rd = p[3];	////����
	}
	else {
		ru = p[3];
		rd = p[2];
	}
	
	std::vector<cv::Point2f> pu;
	pu.push_back(lu);
	pu.push_back(ru);
	pu.push_back(rd);
	pu.push_back(ld);




	cv::Mat rvec;
	cv::Mat tvec;
	Eigen::Vector3d tv;
	
	
	cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec);
	
	
	cv::cv2eigen(tvec, tv);//���תΪ��ɶ
	
	return tv;
}

float KAL::keep_pi(float angle)
{
	if (angle > 180.0)
	{
		return -(360.0 - angle);
	}
	else if (angle < -180.0)
	{
		return 360.0 + angle;
	}
	else
	{
		return angle;
	}
}
