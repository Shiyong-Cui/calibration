// Joint.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <opencv2/opencv.hpp>

#include "BaslerRGB.h"
#include "BaslerTOF.h"
#include "JointCalibration.h"
#include "CameraRig.h"

// capture images for calibration
void CaptureImages(const string& rgb_folder, const string& tof_folder);

// calibration
void Calibration(const string& rgb_folder, const string& tof_folder);

// test projection
void ProjectPoints();

// an example how to use the calibration 
string left_paths[] = {R"(C:\Users\Cui\source\repos\calib_stereo\calib_imgs\1\left)"};
string right_paths[] = {R"(C:\Users\Cui\source\repos\calib_stereo\calib_imgs\1\right)"};

int main()
{
	// folders to save the captured images
	int dataset_index = 0;
	string rgb_folder = left_paths[dataset_index];
	string tof_folder = right_paths[dataset_index];

	// CaptureImages(rgb_folder, tof_folder);
	// Calibration(rgb_folder, tof_folder);
	ProjectPoints();
	return 0;
}

void CaptureImages(const string& rgb_folder, const string& tof_folder)
{
	int frame = 0;
	cv::namedWindow("OpenCV Display Window", cv::WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO
	cv::namedWindow("ToF Intensity", cv::WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO

	sensors::BaslerTOF tofCamera;
	sensors::BaslerRGB rgbCamera;
	rgbCamera.start();
	tofCamera.start();

	while (rgbCamera.isGrabbing())
	{
		bool brgb = rgbCamera.grabImages();
		bool btof = tofCamera.grabImages();
		if (brgb && btof) {
			cv::Mat rgb = rgbCamera.getRGBMap();
			double factor = 0.3;
			cv::Mat outImg;
			cv::resize(rgb, outImg, cv::Size(), factor, factor);
			cv::imshow("OpenCV Display Window", outImg);
			cv::Mat intensity = tofCamera.getIntensityMap();
			cv::imshow("ToF Intensity", intensity);

			while (int c = cv::waitKey(10))
			{
				if (c == 's')
				{
					// save ToF Intensity image
					ostringstream stringStream;
					stringStream << rgb_folder << "/frame_" << frame << ".jpg";
					std::string  file_path = stringStream.str();
					bool bwrite_jpg = cv::imwrite(file_path, rgb);
					if (!bwrite_jpg)
						cout << "cannot save rgb images." << endl;

					stringStream.str("");
					stringStream.clear();
					stringStream << tof_folder << "/frame_" << frame << ".png";
					file_path = stringStream.str();
					bool bwrite_png = cv::imwrite(file_path, intensity);
					if (!bwrite_png)
						cout << "cannot save intensity images." << endl;

					frame++;
				}
				else if (c == 'q')
				{
					rgbCamera.stop();
				}
				break;
			}
		}
	}

	tofCamera.stop();
	rgbCamera.stop();

}

void Calibration(const string& rgb_folder, const string& tof_folder)
{
	int num_im = 18; // 29;
	vector<string> left_images(num_im), right_images(num_im);
	for (int i = 0; i < num_im; i++)
	{
		left_images[i] = rgb_folder + "\\frame_" + std::to_string(i) + ".jpg";
		right_images[i] = tof_folder + "\\frame_" + std::to_string(i) + ".png";
	}

	// folder to save the calibration parameters
	string res_path = R"(C:\Users\Cui\source\repos\ToFApp\Joint\calib3)";
	const std::vector<int> chessboard_size = { 4, 7 }; //  {6, 9};
	const std::vector<double> chessboard_physi_size = { 35.0, 35.0 };
	JointCalibration calibrator;
	calibrator.Calibration(res_path, left_images, right_images, chessboard_size, chessboard_physi_size, true);

}

void ProjectPoints()
{
	int frame = 0;
	vector<cv::Point2d> projected_points;
	vector<cv::Point3d> original_3d_points;
	const string data_folder = R"(C:\Users\Cui\source\repos\ToFApp\Joint\points\data)";
	const string calib_file = R"(C:\Users\Cui\source\repos\ToFApp\Joint\calib\calib_joint.yml)";
	CameraRig cameras;
	// load the calibration file
	cameras.loadJointCalibration(calib_file);

	// start camera
	cameras.start();
	// capture images
	while (cameras.isGrabing())
	{
		bool bsuccess = cameras.grabData();
		if (bsuccess)
		{
			cv::Mat rgb = cameras.getRGBMap();
			cv::Mat intensity = cameras.getIntensityMap();

			cv::imshow("ToF Intensity", intensity);

			while (int c = cv::waitKey(10))
			{
				if (c == 's')
				{			
					// projection
					cameras.projectPointCloud(projected_points, original_3d_points);
					for (size_t i = 0; i < projected_points.size(); i += 50)
					{
						cv::circle(rgb, projected_points[i], 4, cv::Scalar(0, 0, 255), 4);
					}
				}
				else if (c == 'q')
				{
					cameras.stop();
				}
				break;
			}
		} 
		else
		{
			cerr << "capture data failed!" << endl;
		}
	}

	// stop cameras
	cameras.stop();
}
