#pragma once
#include <string>
#include <iostream>
#include <vector>

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>

#ifndef __BASLERRGB__
#define __BASLERRGB__

using namespace std;
using namespace Pylon;

namespace sensors {

	class BaslerRGB {
	public:
		BaslerRGB() : serialNumber_("40006743"), rgbCamera_(nullptr)
		{
			PylonInitialize();
			// Settings
			formatConverter_.OutputPixelFormat = PixelType_BGR8packed;

		}
		~BaslerRGB() {
			if (rgbCamera_) delete rgbCamera_;
			PylonTerminate();			
		}
		void start() {
			if (!rgbCamera_) {
				// Enumerate Cameras
				DeviceInfoList_t list;
				CTlFactory::GetInstance().EnumerateDevices(list);
				bool camFound = false;
				for (int i = 0; i < list.size(); i++) {
					CDeviceInfo& deviceInfo = list[i];
					if (deviceInfo.GetSerialNumber() == serialNumber_.c_str()) {
						cout << "RGB Camera Found" << endl;
						rgbCamera_ = new CInstantCamera(CTlFactory::GetInstance().CreateDevice(deviceInfo));
						if (!rgbCamera_)
							cout << "cannot instantiate RGB camera." << endl;
						rgbCamera_->Open();
						camFound = true;
						break;
					}
				}

				// Stop if list is empty or serial number mismatch
				if (list.empty())
				{
					throw RUNTIME_EXCEPTION("No cameras found.");
				}
				if (!camFound)
				{
					throw RUNTIME_EXCEPTION("Serial No not found");
				}

				// Start Camera
				rgbCamera_->StartGrabbing();
				cout << "Camera Started" << endl;
			}
			
			if (rgbCamera_ && !rgbCamera_->IsOpen())
			{
				rgbCamera_->Open();
				// Start Camera
				rgbCamera_->StartGrabbing();
				cout << "Camera Started" << endl;
			}

		}

		void stop() {
			if (rgbCamera_->IsOpen()) {
				cout << "Camera Stopped" << endl;
				// Stop the camera
				rgbCamera_->StopGrabbing();
				rgbCamera_->Close();
			}
		}

		bool isGrabbing()
		{
			return rgbCamera_->IsGrabbing();
		}

		bool grabImages() {
			CGrabResultPtr ptrGrabResult;
			if (rgbCamera_->IsGrabbing()) {

				// Wait for an image and then retrieve it. A timeout of 5000 ms is used.
				rgbCamera_->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

				// Image grabbed successfully?
				if (ptrGrabResult->GrabSucceeded())
				{
					// Access the image data.
					formatConverter_.Convert(pylonImage_, ptrGrabResult);
					rgbMap_ = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage_.GetBuffer());
					return true;
				}
				else
				{
					cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
					return false;
				}
			}
			else return false;
		}

		cv::Mat getRGBMap() {
			return rgbMap_;
		}

	private:
		CInstantCamera* rgbCamera_;
		// serial number of the camera that is to be opened
		std::string serialNumber_;

		cv::Mat rgbMap_;
		cv::Size imageSize_;

		CImageFormatConverter formatConverter_;
		CPylonImage pylonImage_;
	};

}
#endif
