#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

class Camera
{
public:
	Camera();
	~Camera();
	void Calibrate(const std::string& res_path, 
		const std::vector<std::string>& images, 
		const std::vector<int>& chessboard_size, 
		const std::vector<double>& chessboard_physi_size);
	const cv::Mat& Camera_matrix() const { return m_camera_matrix; }
	void Camera_matrix(const cv::Mat& val) { m_camera_matrix = val; }
	const cv::Mat& Distortion_coeffs() const { return m_distortion_coeffs; }
	void Distortion_coeffs(const cv::Mat& val) { m_distortion_coeffs = val; }
	void Load(const std::string& res_path);
	void Save(const std::string& file) const;
	// 
private:
	// calibrated parameters
	cv::Mat m_camera_matrix;
	cv::Mat m_distortion_coeffs;
};

