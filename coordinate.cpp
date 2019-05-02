#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <windows.h>

#define M_PI 3.14159265359

void DrawTrackingPosition(float, float, float, float);

struct coodinate {
	int x;
	int y;
};

int main()
{
	float yaw = 0;
	float pre_gyro_y = 0;
	double time = 0;

	LARGE_INTEGER freq;
	LARGE_INTEGER start, end;

	QueryPerformanceFrequency(&freq);

	rs2::pipeline pipe;
	pipe.start();

	const auto fish_right = "Right FishCamera";
	const auto fish_left = "Left FishCamera";

	cv::namedWindow(fish_right, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(fish_left, cv::WINDOW_AUTOSIZE);

	QueryPerformanceCounter(&start);
	while (cv::waitKey(1) < 0) // Application still alive?
	{

		auto frameset = pipe.wait_for_frames();
		rs2::video_frame Left = frameset.get_fisheye_frame(1);
		rs2::video_frame Right = frameset.get_fisheye_frame(2);

		//system("cls");

		const int rw = Right.get_width();
		const int rh = Right.get_height();

		const int lw = Left.get_width();
		const int lh = Left.get_height();

		cv::Mat right_image(cv::Size(rw, rh), CV_8UC1, (void*)Right.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat left_image(cv::Size(lw, lh), CV_8UC1, (void*)Left.get_data(), cv::Mat::AUTO_STEP);

		cv::resize(right_image, right_image, cv::Size(), 0.5, 0.5);
		cv::resize(left_image, left_image, cv::Size(), 0.5, 0.5);

		cv::imshow(fish_right, right_image);
		cv::imshow(fish_left, left_image);

		if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
		{
			QueryPerformanceCounter(&end);
			rs2_vector gyro_sample = gyro_frame.get_motion_data();
			//std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;

			time = static_cast<double>(end.QuadPart - start.QuadPart) / freq.QuadPart;

			yaw += (pre_gyro_y + gyro_sample.y) * time / 2;
			pre_gyro_y = gyro_sample.y;

			if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE)) {
				rs2_pose pose_sample = pose_frame.get_pose_data();
				//std::cout << "Pose:" << pose_sample.translation.x << ", " << pose_sample.translation.y << ", " << pose_sample.translation.z << std::endl;

				DrawTrackingPosition(pose_sample.translation.x, pose_sample.translation.y, pose_sample.translation.z, -yaw);
			}
		}

		QueryPerformanceCounter(&start);
	}
}

void DrawTrackingPosition(float x, float y, float z, float yaw) {
	cv::Mat position = cv::Mat::zeros(200, 200, CV_8UC3);

	cv::line(position, cv::Point(0, 100), cv::Point(200, 100), cv::Scalar(0, 0, 255), 3);
	cv::line(position, cv::Point(100, 0), cv::Point(100, 200), cv::Scalar(0, 255, 0), 3);

	x = 100 * x + 100;
	y = 100 * -y + 100;
	z = 100 * z + 100;

	//yaw = (yaw * 90) * M_PI / 180;
	std::cout << yaw << std::endl;

	coodinate begin;
	coodinate end;

	begin.x = x + -5 * cos(yaw);
	begin.y = z + -5 * sin(yaw);

	end.x = x + 5 * cos(yaw);
	end.y = z + 5 * sin(yaw);

	cv::line(position, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(255, 255, 255), 5);

	cv::resize(position, position, cv::Size(), 2, 2);

	cv::imshow("Position", position);
}