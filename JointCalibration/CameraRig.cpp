#include "pch.h"
#include "CameraRig.h"

CameraRig::CameraRig()
{
}


CameraRig::~CameraRig()
{
}

void CameraRig::start()
{
	rgbCamera.start();
	tofCamera.start();
}

void CameraRig::stop()
{
	tofCamera.stop();
	rgbCamera.stop();
}

void CameraRig::stopRGBCamera()
{
	rgbCamera.stop();
}

void CameraRig::stopToFCamera()
{
	tofCamera.stop();
}

bool CameraRig::isGrabing()
{
	return rgbCamera.isGrabbing();
}

bool CameraRig::grabData()
{
	bool brgb = rgbCamera.grabImages();
	bool btof = tofCamera.grabImages();
	return brgb && btof;
}

cv::Mat CameraRig::getRGBMap()
{
	return rgbCamera.getRGBMap();
}

cv::Mat CameraRig::getIntensityMap()
{
	return tofCamera.getIntensityMap();
}

void CameraRig::loadRGBCameraIntrinsic(const string& rgb_calib_file)
{
	cv::FileStorage fs(rgb_calib_file, cv::FileStorage::READ);
	fs["camera matrix"] >> m_rgb_camera_matrix;
	fs["distortion coefficients"] >> m_rgb_distortion_coeffs;
	fs.release();
}

void CameraRig::loadToFCameraIntrinsic(const string& tof_calib_file)
{
	cv::FileStorage fs(tof_calib_file, cv::FileStorage::READ);
	fs["camera matrix"] >> m_tof_camera_matrix;
	fs["distortion coefficients"] >> m_tof_distortion_coeffs;
	fs.release();
}

void CameraRig::loadJointCalibration(const string& joint_calib_file)
{
	cv::FileStorage fs(joint_calib_file, cv::FileStorage::READ);
	fs["K1"] >> m_rgb_camera_matrix;
	fs["K2"] >> m_tof_camera_matrix;
	fs["D1"] >> m_rgb_distortion_coeffs;
	fs["D2"] >> m_tof_distortion_coeffs;
	fs["R"] >> m_rotation;
	fs["T"] >> m_translation;
	fs["E"] >> m_essential;
	fs["F"] >> m_fundamental;
	fs.release();

}

void CameraRig::writePcdHeader(ostream& o, size_t width, size_t height, bool saveIntensity)
{
	o << "# .PCD v0.7 - Point Cloud Data file format" << endl;
	o << "VERSION 0.7" << endl;
	o << "FIELDS x y z rgb" << endl;
	o << "SIZE 4 4 4";
	if (saveIntensity)
		o << " 4";
	o << endl;

	o << "TYPE F F F";
	if (saveIntensity)
		o << " F";
	o << endl;

	o << "COUNT 1 1 1";
	if (saveIntensity)
		o << " 1";
	o << endl;

	o << "WIDTH " << width << endl;
	o << "HEIGHT " << height << endl;
	o << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	o << "POINTS " << width * height << endl;
	o << "DATA ASCII" << endl;

}

bool CameraRig::savePointCloud(const char* fileName)
{
	const BufferParts& parts = tofCamera.getData();
	if (parts.empty())
	{
		cerr << "No valid image data." << endl;
		return false;
	}

	// If the point cloud is enabled, the first part always contains the point cloud data.
	if (parts[0].dataFormat != PFNC_Coord3D_ABC32f)
	{
		cerr << "Unexpected data format for the first image part. Coord3D_ABC32f is expected." << endl;
		return false;
	}

	const bool saveIntensity = parts.size() > 1;
	if (saveIntensity && parts[1].dataFormat != PFNC_Mono16)
	{
		cerr << "Unexpected data format for the second image part. Mono 16 is expected." << endl;
		return false;
	}

	ofstream o(fileName);
	if (!o)
	{
		cerr << "Error:\tFailed to create file " << fileName << endl;
		return false;
	}

	cout << "Writing point cloud to file " << fileName << "...";
	CToFCamera::Coord3D *pPoint = (CToFCamera::Coord3D*) parts[0].pData;
	uint16_t *pIntensity = saveIntensity ? (uint16_t*)parts[1].pData : NULL;
	const size_t nPixel = parts[0].width * parts[0].height;

	writePcdHeader(o, parts[0].width, parts[0].height, saveIntensity);

	for (size_t i = 0; i < nPixel; ++i)
	{
		// Check if there are valid 3D coordinates for that pixel.
		if (pPoint->IsValid())
		{
			o.precision(0);  // Coordinates will be written as whole numbers.

			// Write the coordinates of the next point. Note: Since the coordinate system
			// used by the CloudCompare tool is different from the one used by the ToF camera, 
			// we apply a 180-degree rotation around the x-axis by writing the negative 
			// values of the y and z coordinates.
			o << std::fixed << pPoint->x << ' ' << pPoint->y << ' ' << pPoint->z;

			if (saveIntensity)
			{
				// Save the intensity as an RGB value.
				uint8_t gray = *pIntensity >> 8;
				uint32_t rgb = (uint32_t)gray << 16 | (uint32_t)gray << 8 | (uint32_t)gray;
				// The point cloud library data format represents RGB values as floats. 
				float fRgb = *(float*)&rgb;
				o.unsetf(ios_base::floatfield); // Switch to default float formatting
				o.precision(9); // Intensity information will be written with highest precision.
				o << ' ' << fRgb << endl;
			}
		}
		else
		{
			o << "nan nan nan 0" << endl;
		}
		pPoint++;
		pIntensity++;
	}
	o.close();
	cout << "done." << endl;
	return true;
}

inline const GenTLConsumerImplHelper::CToFCamera::Coord3D* CameraRig::getPointCloud()
{
	return tofCamera.getPointCloud();
}

inline const cv::Size CameraRig::getRangeMapSize()
{
	return tofCamera.getImageSize();
}

cv::Point3d CameraRig::getPosition(const cv::Rect2d& boundingBox)
{
	vector<cv::Point2d> all_projected_points;
	vector<cv::Point3d> original_3d_points;
	projectPointCloud(all_projected_points, original_3d_points);

	vector<cv::Point2d> seleted_projected_points;
	vector<cv::Point3d> selected_3d_points;
	locatePointsInRect(all_projected_points, original_3d_points, boundingBox, seleted_projected_points, selected_3d_points);

	cv::Point3d position;
	computePosition(selected_3d_points, position);

	return position;
}

void CameraRig::projectPointCloud(vector<cv::Point2d>& points, vector<cv::Point3d>& original_3d_points)
{
	if (points.size()) points.clear();
	if (original_3d_points.size()) original_3d_points.clear();
 	const GenTLConsumerImplHelper::CToFCamera::Coord3D* point = tofCamera.getPointCloud();
	cv::Size rangeMapSize = tofCamera.getImageSize();
	int nPixel = rangeMapSize.width * rangeMapSize.height;
	// compute inverse
	cv::Mat temp_upper, temp_all;
	cv::Mat temp = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 1.0);
	cv::hconcat(m_rotation, m_translation, temp_upper);
	cv::vconcat(temp_upper, temp, temp_all);

	cv::Mat reverseMat = temp_all.inv();
	cv::Mat rotation2 = reverseMat(cv::Range(0, 3), cv::Range(0, 3));
	cv::Mat translation2 = reverseMat(cv::Range(0, 3), cv::Range(3, 4));

	for (size_t i = 0; i < nPixel; ++i)
	{
		if (point->IsValid())
		{			
			cv::Mat point3D = (cv::Mat_<double>(3, 1) << point->x, point->y, point->z);
			// cv::Mat point2D = m_rgb_camera_matrix * (m_rotation.inv() * (point3D - m_translation));
			cv::Mat point2D = m_rgb_camera_matrix * (m_rotation.t() * (point3D - m_translation));
			double u = point2D.at<double>(0, 0) / point2D.at<double>(2, 0);
			double v = point2D.at<double>(1, 0) / point2D.at<double>(2, 0);
			points.push_back(cv::Point2d(u, v));
			original_3d_points.push_back(cv::Point3d(point->x, point->y, point->z));
		}
		point++;
	}
}

void CameraRig::locatePointsInRect(const vector<cv::Point2d>& all_projected_points,
	const vector<cv::Point3d>& original_3d_points, 
	const cv::Rect2d& boundingBox, 
	vector<cv::Point2d>& seleted_projected_points, 
	vector<cv::Point3d>& selected_3d_points)
{
	if (all_projected_points.size() != original_3d_points.size())
		cout << "the number of points does not match in fuction CameraRig::locatePointsWithRect." << endl;

	for (size_t i = 0; i < all_projected_points.size(); ++i)
	{
		if (boundingBox.contains(all_projected_points[i]))
		{
			seleted_projected_points.push_back(all_projected_points[i]);
			selected_3d_points.push_back(original_3d_points[i]);
		}
	}
}

void CameraRig::computePosition(const vector<cv::Point3d>& selected_3d_points, cv::Point3d& position)
{
	size_t nbPoints = selected_3d_points.size();
	double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
	for (size_t i = 0; i < nbPoints; ++i)
	{
		sum_x += selected_3d_points[i].x;
		sum_y += selected_3d_points[i].y;
		sum_z += selected_3d_points[i].z;
	}
	position.x = sum_x / nbPoints;
	position.y = sum_y / nbPoints;
	position.z = sum_z / nbPoints;
}

void CameraRig::MatType(const cv::Mat& inputMat)
{
	int inttype = inputMat.type();

	string r, a;
	uchar depth = inttype & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (inttype >> CV_CN_SHIFT);
	switch (depth) {
	case CV_8U:  r = "8U";   a = "Mat.at<uchar>(y,x)"; break;
	case CV_8S:  r = "8S";   a = "Mat.at<schar>(y,x)"; break;
	case CV_16U: r = "16U";  a = "Mat.at<ushort>(y,x)"; break;
	case CV_16S: r = "16S";  a = "Mat.at<short>(y,x)"; break;
	case CV_32S: r = "32S";  a = "Mat.at<int>(y,x)"; break;
	case CV_32F: r = "32F";  a = "Mat.at<float>(y,x)"; break;
	case CV_64F: r = "64F";  a = "Mat.at<double>(y,x)"; break;
	default:     r = "User"; a = "Mat.at<UKNOWN>(y,x)"; break;
	}
	r += "C";
	r += (chans + '0');
	cout << "Mat is of type " << r << " and should be accessed with " << a << endl;

}

