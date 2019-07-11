#include "pch.h"
#include "JointCalibration.h"


JointCalibration::JointCalibration()
{
}


JointCalibration::~JointCalibration()
{
}

void JointCalibration::CalibrateLeft(const std::string& res_path, const std::vector<std::string>& images, 
	const std::vector<int>& chessboard_size, const std::vector<double>& chessboard_physi_size)
{
	left_camera.Calibrate(res_path, images, chessboard_size, chessboard_physi_size);
}

void JointCalibration::LoadLeft(const std::string& cam_file)
{
	left_camera.Load(cam_file);
}

void JointCalibration::LoadRight(const std::string& cam_file)
{
	right_camera.Load(cam_file);
}

void JointCalibration::SaveLeft(const std::string& cam_file)
{
	left_camera.Save(cam_file);
}

void JointCalibration::SaveRight(const std::string& cam_file)
{
	right_camera.Save(cam_file);
}

void JointCalibration::Load(const std::string& joint_cam_file)
{

}

void JointCalibration::Save(const std::string& joint_cam_file)
{
	// save the results
	cv::FileStorage fs(joint_cam_file, cv::FileStorage::WRITE);
	fs << "K1" << left_camera.Camera_matrix();
	fs << "K2" << right_camera.Camera_matrix();
	fs << "D1" << left_camera.Distortion_coeffs();
	fs << "D2" << right_camera.Distortion_coeffs();
	fs << "R" << rotation;
	fs << "T" << translation;
	fs << "E" << essential;
	fs << "F" << fundamental;
	fs.release();
}

void JointCalibration::CalibrateRight(const std::string& res_path, 
	const std::vector<std::string>& images, 
	const std::vector<int>& chessboard_size, 
	const std::vector<double>& chessboard_physi_size)
{
	right_camera.Calibrate(res_path, images, chessboard_size, chessboard_physi_size);
}

void JointCalibration::Calibration(const std::string& res_path, 
	const std::vector<std::string>& left_imgs, 
	const std::vector<std::string>& right_imgs, 
	const std::vector<int>& chessboard_size, 
	const std::vector<double>& chessboard_physi_size, 
	bool single_calib /*= true*/)
{
	left_images = left_imgs;
	right_images = right_imgs;

	if (single_calib)
	{
		// calibration left
		std::string calib_left = res_path + "\\calib_left.yml";
		left_camera.Calibrate(calib_left, left_images, chessboard_size, chessboard_physi_size);
		// calibration right
		std::string calib_right = res_path + "\\calib_right.yml";
		right_camera.Calibrate(calib_right, right_images, chessboard_size, chessboard_physi_size);
	}

	if (left_images.size() == right_images.size())
	{
		std::vector< std::vector< cv::Point2f > > left_img_points;
		std::vector< std::vector< cv::Point2f > > right_img_points;
		std::vector< std::vector< cv::Point3f > > object_points;

		findCorrespondence(chessboard_size, chessboard_physi_size, 
			left_img_points, right_img_points, object_points);
		
		std::cout << "starting calibration..." << std::endl;
		stereoCalibrate(object_points, left_img_points, right_img_points, 
			left_camera.Camera_matrix(), left_camera.Distortion_coeffs(), 
			right_camera.Camera_matrix(), right_camera.Distortion_coeffs(), 
			left_img_size, rotation, translation, essential, fundamental);

		std::cout << "finishing calibration." << std::endl;

		// save the calibration results
		std::string calib_joint = res_path + "\\calib_joint.yml";
		Save(calib_joint);
	} 
	else
	{
		std::cout << "the number of left and right images are not the same." << std::endl;
	}	
}

void JointCalibration::findCorrespondence(const std::vector<int>& chessboard_size, 
	const std::vector<double>& chessboard_physi_size, 
	std::vector< std::vector< cv::Point2f > >&left_img_points, 
	std::vector< std::vector< cv::Point2f > >&right_img_points, 
	std::vector< std::vector< cv::Point3f > >& object_points)
{
	int num_imgs = left_images.size();
	int board_width = chessboard_size[1];
	int board_height = chessboard_size[0];

	cv::Size board_size(board_width, board_height);
	int board_n = board_width * board_height;
	cv::Mat img_left, img_right, gray_left, gray_right;
	std::vector< cv::Point2f > left_corners, right_corners;

	// form a template of a set of object points
	std::vector< cv::Point3f > obj;
	for (int i = 0; i < board_height; i++)
		for (int j = 0; j < board_width; j++) 	{
			obj.push_back(cv::Point3f((float)j * chessboard_physi_size[1], (float)i * chessboard_physi_size[0], 0));
		}

	for (int i = 0; i < num_imgs; i++)
	{
		img_left = cv::imread(left_images[i], cv::IMREAD_COLOR);
		img_right = cv::imread(right_images[i], cv::IMREAD_COLOR);
		cvtColor(img_left, gray_left, cv::COLOR_BGR2GRAY);
		cvtColor(img_right, gray_right, cv::COLOR_BGR2GRAY);
		left_img_size = img_left.size();
		right_img_size = img_right.size();

		bool found_left = false, found_right = false;
		// find the corners from both left and right images
		found_left = cv::findChessboardCorners(img_left, board_size, left_corners,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		found_right = cv::findChessboardCorners(img_right, board_size, right_corners,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

		if (found_left)
		{
			cv::cornerSubPix(gray_left, left_corners, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			cv::drawChessboardCorners(gray_left, board_size, left_corners, found_left);
		}
		if (found_right)
		{
			cv::cornerSubPix(gray_right, right_corners, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			cv::drawChessboardCorners(gray_right, board_size, right_corners, found_right);
		}

		if (found_left && found_right) {
			std::cout << i << ". Found corners!" << std::endl;
			left_img_points.push_back(left_corners);
			right_img_points.push_back(right_corners);
			object_points.push_back(obj);
		}
	}
}


