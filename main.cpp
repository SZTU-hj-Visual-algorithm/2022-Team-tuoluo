#include "camera.h"
#include "ArmorDetector.hpp"
#include <tuoluo_detect.h>
#include <tuoluo_variables.h>

using namespace cv;

int main()
{
	
	auto camera_warper = new Camera;
	Mat src;

	ArmorDetector shibie = ArmorDetector();
	shibie.enemy_color = RED;
	ShootTuoluo shoot_tuoluo;
	if (camera_warper->init())
	{
		while (waitKey(10)!=27)
		{
			double time_count = (double)getTickCount();
			camera_warper->read_frame_rgb();
			src = cv::cvarrToMat(camera_warper->ipiimage).clone();

			RotatedRect mubiao;
			mubiao = shibie.getTargetAera(src, 0, 0);

            TuoluoData is_tuoluo;
            
            
            is_tuoluo = shoot_tuoluo.getTuoluoData(src, shibie.BestArmor);

			Point2f ps[4];
			mubiao.points(ps);
			for (int i = 0; i < 4; i++) {
				line(src, ps[i], ps[(i + 1) % 4], CV_RGB(255, 0, 0));
			}
			double fps = 1 / (((double)getTickCount() - time_count) / getTickFrequency());
			char strN[10];
			sprintf(strN, "%.4f", fps);
			putText(src, strN, Point(30, 30), 5, 1.5, Scalar(0, 0, 255), 2);
			imshow("src",src);
			camera_warper->release_data();
		}
		camera_warper->~Camera();
		return 0;
	}
	else
	{
		printf("No camera!!");
		return 0;
	}
	
}
