#pragma once
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

class MonoCalibration
{
public:
	MonoCalibration();
	MonoCalibration(std::vector<std::string> files, std::vector<int> size, std::vector<double> physi_size);
	~MonoCalibration();
	void Calibrate(const std::string& res_path);
	void SaveCameraParameters(const std::string& file) const;
	void LoadCalibration(const std::string& res_path);
	std::vector<std::string> Chessboard_img_fname() const { return chessboard_img_fnames; } 
	void Chessboard_img_fname(const std::vector<std::string>& val) { chessboard_img_fnames = val; }
	std::vector<int> Chessboard_size() const { return chessboard_size; }
	void Chessboard_size(const std::vector<int>& val) { chessboard_size = val; }
	std::vector<double> Chessboard_phys_size() const { return chessboard_phys_size; }
	void Chessboard_phys_size(const std::vector<double>& val) { chessboard_phys_size = val; }
	cv::Mat Camera_matrix() const { return m_camera_matrix; }
	cv::Mat Distortion_coeffs() const { return m_distortion_coeffs; }
private:
	double computeReprojectionErrors(const std::vector< std::vector< cv::Point3f > >& objectPoints,
		const std::vector< std::vector< cv::Point2f > >& imagePoints,
		const std::vector< cv::Mat >& rvecs, const std::vector< cv::Mat >& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

	// input images for calibration
	std::vector<std::string> chessboard_img_fnames;
	// the number and column of the grids on the board
	std::vector<int> chessboard_size;
	// the physical size of each grid on the board: [height, width]
	std::vector<double> chessboard_phys_size;
	// calibrated parameters
	cv::Mat m_camera_matrix;
	cv::Mat m_distortion_coeffs;
};

