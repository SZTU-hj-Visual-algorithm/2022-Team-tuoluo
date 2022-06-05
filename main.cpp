#include "camera.h"
#include "ArmorDetector.hpp"
#include "serialport.h"
#include <tuoluo_Detect.h>
#include <tuoluo_variables.h>

using namespace cv;

int main()
{
	
	auto camera_warper = new Camera;
	Mat src;
	VisionData vdata;

    ShootTuoluo realTuoluo;

	SerialPort port("/dev/ttyUSB0");
	port.initSerialPort();

	ArmorDetector shibie = ArmorDetector();
	shibie.enemy_color = RED;

	if (camera_warper->init())
	{
		while (waitKey(10)!=27)
		{
			double time_count = (double)getTickCount();
			camera_warper->read_frame_rgb();
			src = cv::cvarrToMat(camera_warper->ipiimage).clone();

			RotatedRect mubiao;
			mubiao = shibie.getTargetAera(src, 0, 0);

			Point2f ps[4];
			mubiao.points(ps);
			for (int i = 0; i < 4; i++) {
				line(src, ps[i], ps[(i + 1) % 4], CV_RGB(255, 0, 0));
			}

            TLdata = realTuoluo.getTuoluoData(src);
            if(TLdata.isTuoluo)
            {
                realTuoluo.drawImage(src, TLtuoluo.angle, (mubiao.center.x, mubiao.center.y), TLtuoluo.R, float
                armorWidth)
            }

            }
            double timing  = ((double)getTickCount() - time_count) / getTickFrequency();
			double fps = 1 / timing;
            double cartime = timeing*1000.0;

			char strN[10];
			sprintf(strN, "%.4f", fps);
			putText(src, strN, Point(30, 30), 5, 1.5, Scalar(0, 0, 255), 2);
			imshow("src",src);


			int lin_is_get;//是否获取到c板绝对角
			float lin[4];
			int mode_temp;
			lin_is_get = port.get_Mode1(mode_temp, lin[0], lin[1], lin[2], lin[3]);  //第一个参数为模式，第二个为pitch，yaw，roll，ball_speed

            		CarData carDatas;
            		carDatas.pitch = lin[1];
            		carDatas.yaw = lin[2];
            		carDatas.BeginToNowTime = cartime;

			float pitch,yaw;
			vdata = { pitch, yaw, 0x31 };
			vdata = { pitch, yaw, 0x32 };  //没识别的时候
			port.TransformData(vdata);
			port.send();

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
