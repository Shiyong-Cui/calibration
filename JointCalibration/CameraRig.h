#pragma once

#include "BaslerRGB.h"
#include "BaslerTOF.h"

class CameraRig
{
public:
	CameraRig();
	~CameraRig();
	// start the camera rig
	void start();
	// stop the camera rig
	void stop();
	// check whether cameras are capturing
	bool isGrabing();
	// capture data
	bool grabData();
	// get the RGB image
	cv::Mat getRGBMap();
	// get the intensity map of the ToF camera
	cv::Mat getIntensityMap();
	// load the RGB intrinsic parameters
	void loadRGBCameraIntrinsic(const string& rgb_calib_file);
	// load the ToF intrinsic parameters
	void loadToFCameraIntrinsic(const string& tof_calib_file);
	// load the calibration parameters
	void loadJointCalibration(const string& joint_calib_file);
	// save the point cloud
	bool savePointCloud(const char* fileName);
	// get a pointer to the 3D point cloud
	const CToFCamera::Coord3D* getPointCloud();
	// get the dimension of the range map
	const cv::Size getRangeMapSize();
	// bounding box of the target in the RGB camera
	cv::Point3d getPosition(const cv::Rect2d& boundingBox);

	void stopRGBCamera();
	void stopToFCamera();
	void writePcdHeader(ostream& o, size_t width, size_t height, bool saveIntensity);
	void projectPointCloud(vector<cv::Point2d>& points, vector<cv::Point3d>& original_3d_points);
	void locatePointsInRect(const vector<cv::Point2d>& all_projected_points, 
		const vector<cv::Point3d>& original_3d_points,
		const cv::Rect2d& boundingBox, 
		vector<cv::Point2d>& seleted_projected_points, 
		vector<cv::Point3d>& selected_3d_points);
	void computePosition(const vector<cv::Point3d>& selected_3d_points, cv::Point3d& position);

	// for debug
	void MatType(const cv::Mat& inputMat);

private:
	sensors::BaslerTOF tofCamera;
	sensors::BaslerRGB rgbCamera;

	// RGB intrinsic
	cv::Mat m_rgb_camera_matrix;
	cv::Mat m_rgb_distortion_coeffs;

	// ToF intrinsic
	cv::Mat m_tof_camera_matrix;
	cv::Mat m_tof_distortion_coeffs;

	// calibrated parameters
	cv::Mat m_rotation;
	cv::Mat m_translation;
	cv::Mat m_essential;
	cv::Mat m_fundamental;
};

