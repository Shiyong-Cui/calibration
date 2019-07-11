#pragma once

#include "Camera.h"

class JointCalibration
{
public:
	JointCalibration();
	~JointCalibration();
	// calibrate intrinsics
	void CalibrateLeft(const std::string& res_path,
		const std::vector<std::string>& images,
		const std::vector<int>& chessboard_size,
		const std::vector<double>& chessboard_physi_size);
	void CalibrateRight(const std::string& res_path,
		const std::vector<std::string>& images,
		const std::vector<int>& chessboard_size,
		const std::vector<double>& chessboard_physi_size);
	// in case of calibrated intrinsics, just load them
	void LoadLeft(const std::string& cam_file);
	void LoadRight(const std::string& cam_file);
	// save intrinsics
	void SaveLeft(const std::string& cam_file);
	void SaveRight(const std::string& cam_file);
	// load and save joint calibration
	void Load(const std::string& joint_cam_file);
	void Save(const std::string& joint_cam_file);
	// joint calibration
	void Calibration(const std::string& res_path,
		const std::vector<std::string>& left_images,
		const std::vector<std::string>& right_images,
		const std::vector<int>& chessboard_size,
		const std::vector<double>& chessboard_physi_size,
		bool single_calib = true);
	// get the cameras
	const Camera& LeftCamera() const { return left_camera; }
	const Camera& RightCamera() const { return right_camera; }

private:
	// find the corresponding points from both left and right images
	void findCorrespondence(const std::vector<int>& chessboard_size,
		const std::vector<double>& chessboard_physi_size,
		std::vector< std::vector< cv::Point2f > >&left_img_points,
		std::vector< std::vector< cv::Point2f > >&right_img_points,
		std::vector< std::vector< cv::Point3f > >& object_points);

	// cameras
	Camera left_camera;
	Camera right_camera;
	// image size
	cv::Size left_img_size;
	cv::Size right_img_size;
	// images from left and right camera
	std::vector<std::string> left_images;
	std::vector<std::string> right_images;
	// calibrated parameters
	cv::Mat rotation;
	cv::Mat translation;
	cv::Mat essential;
	cv::Mat fundamental;
};

