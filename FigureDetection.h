/*
 *
 * (c) domzigm 2016 - GPLv3
 * https://github.com/domzigm/mt
 * 
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <stdint.h>

namespace mt
{

struct figureDescription
{
	cv::Scalar lowerThreshold;
	cv::Scalar upperThreshold;
};

uint8_t locateFigure(figureDescription& desc, cv::Mat& srcimage, cv::Rect& rect);

}
#pragma once
