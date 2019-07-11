#include "pch.h"
#include "MonoCalibration.h"
#include <iostream>


MonoCalibration::MonoCalibration()
{
}

MonoCalibration::MonoCalibration(std::vector<std::string> files, std::vector<int> size, 
	std::vector<double> physi_size)
{
	if (chessboard_img_fnames.size() < 1)
		std::cout << "no chess board images for calibration." << std::endl;
	else
		chessboard_img_fnames = files;

	if (size.size() < 2)
		std::cout << "size should a vector of two int numbers." << std::endl;
	else
		chessboard_size = size;

	if (physi_size.size() < 2)
		std::cout << "physi_size should be a vector of two double numbers." << std::endl;
	else
		chessboard_phys_size = physi_size;

}

MonoCalibration::~MonoCalibration()
{
}

void MonoCalibration::Calibrate(const std::string& res_path)
{
	int chess_rows = chessboard_size[0];
	int chess_cols = chessboard_size[1];

	cv::Size board_sz = cv::Size(chess_cols, chess_rows);

	std::vector< std::vector<cv::Point2f> > image_points;
	std::vector< std::vector<cv::Point3f> > object_points;

	std::vector<cv::Point3f> objectCorners;

	for (int i = 0; i < chess_rows; i++) {
		for (int j = 0; j < chess_cols; j++) {
			objectCorners.push_back(cv::Point3f(j*chessboard_phys_size[0], i*chessboard_phys_size[1], 0.0f));
		}
	}

	cv::Size image_size;
	for (unsigned int i = 0; i < chessboard_img_fnames.size(); i++)
	{
		// Read the file
		cv::Mat image = cv::imread(chessboard_img_fnames[i], cv::IMREAD_COLOR);

		// convert to gray scale image
		cv::Mat imageGray;
		cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
		image_size = imageGray.size();

		std::vector<cv::Point2f> imageCorners;

		//do a sharpen filter for the large resolution image
		int height = imageGray.rows;
		int width = imageGray.cols;
		cv::Mat shrink;
		double factor = 0.6;
		int max_im_size = 1500;

		if (width > max_im_size) {
			// scale the image
			cv::resize(imageGray, shrink, cv::Size(), factor, factor, cv::INTER_LINEAR);
		}
		else {
			shrink = imageGray;
		}

		bool found = findChessboardCorners(shrink, board_sz, imageCorners,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FILTER_QUADS);

		if (found) {
			std::cout << chessboard_img_fnames[i] << "  True" << std::endl;

			if (width > max_im_size) {
				for (int i = 0; i < imageCorners.size(); i++)
				{      
					// scale back 
					imageCorners[i] /= factor;
				}
			}

			// Get sub-pixel accuracy on the corners
			cornerSubPix(imageGray, imageCorners,
				cv::Size(5, 5),
				cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::MAX_ITER +
					cv::TermCriteria::EPS,
					30,		// max number of iterations
					0.1)); // min accuracy

			image_points.push_back(imageCorners);
			object_points.push_back(objectCorners);
		}
		else {
			std::cout << chessboard_img_fnames[i] << "  False" << std::endl;
		}
	}

	//Output rotations and translations
	std::vector<cv::Mat> rvecs, tvecs;
	int flag = 0;
	flag |= cv::CALIB_FIX_K4;
	flag |= cv::CALIB_FIX_K5;
	double RMS = calibrateCamera(object_points, image_points, image_size, 
		m_camera_matrix, m_distortion_coeffs, rvecs, tvecs, flag);
	std::cout << "RMS error is " << RMS << std::endl;

	if (!res_path.empty()) 
		SaveCameraParameters(res_path);

	double reprojection_error = computeReprojectionErrors(object_points, image_points, 
		rvecs, tvecs, m_camera_matrix, m_distortion_coeffs);
	std::cout << "Calibration error: " << reprojection_error << std::endl;

	std::cout << "Camera matrix: " << std::endl;
	std::cout << m_camera_matrix << std::endl;
	std::cout << "Distortion coefficients: " << std::endl;
	std::cout << m_distortion_coeffs << std::endl;

}

void MonoCalibration::SaveCameraParameters(const std::string& file) const
{
	// write Mat to file
	cv::FileStorage fs(file, cv::FileStorage::WRITE);
	fs << "camera matrix" << m_camera_matrix;
	fs << "distortion coefficients" << m_distortion_coeffs;
	fs.release();
}

void MonoCalibration::LoadCalibration(const std::string& res_path)
{
	// load from file
	cv::FileStorage fs(res_path, cv::FileStorage::READ);
	fs["camera matrix"] >> m_camera_matrix;
	fs["distortion coefficients"] >> m_distortion_coeffs;
	fs.release();
}

double MonoCalibration::computeReprojectionErrors(const std::vector< std::vector< cv::Point3f > >& objectPoints, 
	const std::vector< std::vector< cv::Point2f > >& imagePoints, 
	const std::vector< cv::Mat >& rvecs, 
	const std::vector< cv::Mat >& tvecs, 
	const cv::Mat& cameraMatrix, 
	const cv::Mat& distCoeffs)
{
	std::vector< cv::Point2f > imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	std::vector< float > perViewErrors;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i) {
		projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}
