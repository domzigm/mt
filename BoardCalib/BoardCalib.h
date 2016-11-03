/*
*
* (c) domzigm 2016 - GPLv3
* https://github.com/domzigm/mt
*
*/

#pragma once

#include <string>
#include <opencv2/opencv.hpp>

struct rectangle_t
{
	cv::Rect2f rect;
	cv::Scalar color;
	std::string name;
};
