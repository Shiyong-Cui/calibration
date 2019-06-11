#include "pch.h"
#include "Camera.h"
#include "MonoCalibration.h"

Camera::Camera()
{
}


Camera::~Camera()
{
}

void Camera::Calibrate(const std::string& res_path, const std::vector<std::string>& images, 
	const std::vector<int>& chessboard_size, const std::vector<double>& chessboard_physi_size)
{
	MonoCalibration calibrator;
	calibrator.Chessboard_img_fname(images);
	calibrator.Chessboard_size(chessboard_size);
	calibrator.Chessboard_phys_size(chessboard_physi_size);
	calibrator.Calibrate(res_path);
	m_camera_matrix = calibrator.Camera_matrix();
	m_distortion_coeffs = calibrator.Distortion_coeffs();
}

void Camera::Load(const std::string& res_path)
{
	// load from file
	cv::FileStorage fs(res_path, cv::FileStorage::READ);
	fs["camera matrix"] >> m_camera_matrix;
	fs["distortion coefficients"] >> m_distortion_coeffs;
	fs.release();
}

void Camera::Save(const std::string& file) const
{
	// write Mat to file
	cv::FileStorage fs(file, cv::FileStorage::WRITE);
	fs << "camera matrix" << m_camera_matrix;
	fs << "distortion coefficients" << m_distortion_coeffs;
	fs.release();
}
