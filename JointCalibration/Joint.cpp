// Joint.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <opencv2/opencv.hpp>

#include "BaslerRGB.h"
#include "BaslerTOF.h"
#include "JointCalibration.h"

void CaptureImages(const string& rgb_folder, const string& tof_folder);

// an example how to use the calibration 

int main()
{
	// folders to save the captured images
	string rgb_folder = R"(C:\Users\Cui\source\repos\ToFApp\Joint\images)";
	string tof_folder = R"(C:\Users\Cui\source\repos\ToFApp\Joint\intensity)";

	CaptureImages(rgb_folder, tof_folder);

	int num_im = 11;
	vector<string> left_images(num_im), right_images(num_im);
	for (int i = 0; i < num_im; i++)
	{
		left_images[i] = rgb_folder + "\\frame_" + std::to_string(i) + ".jpg";
		right_images[i] = tof_folder + "\\frame_" + std::to_string(i) + ".png";
	}

	// folder to save the calibration parameters
	string res_path = R"(C:\Users\Cui\source\repos\ToFApp\Joint\calib)";
	const std::vector<int> chessboard_size = { 5, 8 };
	const std::vector<double> chessboard_physi_size = { 24.33, 24.33 };
	JointCalibration calibrator;
	calibrator.Calibration(res_path, left_images, right_images, chessboard_size, chessboard_physi_size, true);
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
			cv::imshow("OpenCV Display Window", rgb);
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

