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

//! See https://en.wikipedia.org/wiki/HSL_and_HSV
struct hsvPlanes
{
	cv::Scalar loThreshold_Left;
	cv::Scalar hiThreshold_Left;
	cv::Scalar loThreshold_Right;
	cv::Scalar hiThreshold_Right;
};

void inHsvRange(cv::Mat& srcImage, hsvPlanes& desc, cv::Mat& dstImage);

void rectToRelative(cv::Mat& mapping, cv::Rect& input_rect, cv::Rect2f& output_rect);
void rectToAbsolute(cv::Mat& mapping, cv::Rect2f& input_rect, cv::Rect& output_rect);

void morphOps(cv::Mat& binaryImage);

}
