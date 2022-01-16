#ifndef CV_UTILS_H_
#define CV_UTILS_H_

#include <opencv2/opencv.hpp>

using namespace cv;

void cvVersion()
{
	std::cout << "OpenCV version : " << CV_VERSION << std::endl;
	std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
	std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;
	std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;
}

#endif