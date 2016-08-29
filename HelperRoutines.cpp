/*
 *
 * (c) domzigm 2016 - GPLv3
 * https://github.com/domzigm/mt
 * 
 */

#include "HelperRoutines.h"

using namespace cv;

namespace mt
{

void inHsvRange(cv::Mat& srcImage, hsvPlanes& desc, cv::Mat& dstImage)
{
	cv::Mat left;
	cv::Mat right;

	inRange(srcImage, desc.loThreshold_Left, desc.hiThreshold_Left, left);

	// If right low == right high threshold, ignore second part
	if (desc.loThreshold_Right[0] != desc.hiThreshold_Right[0]) {

		inRange(srcImage, desc.loThreshold_Right, desc.hiThreshold_Right, right);
		dstImage = left | right;
	}
	else {
		
		dstImage = left;
	}
}

//! Convert absolute (x,y) rect values into relative values
void rectToRelative(Mat& mapping, Rect& input_rect, Rect2f& output_rect)
{
	output_rect.x = (float)(input_rect.x / mapping.cols);
	output_rect.y = (float)(input_rect.y / mapping.rows);
	output_rect.width = (float)(input_rect.width / mapping.cols);
	output_rect.height = (float)(input_rect.height / mapping.rows);
}

//! Convert relative rect values into absolute (x,y) values
void rectToAbsolute(Mat& mapping, Rect2f& input_rect, Rect& output_rect)
{
	output_rect.x = (int)(input_rect.x * mapping.cols);
	output_rect.y = (int)(input_rect.y * mapping.rows);
	output_rect.width = (int)(input_rect.width * mapping.cols);
	output_rect.height = (int)(input_rect.height * mapping.rows);
}

void morphOps(Mat& binaryImage)
{
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(7, 7));

	erode(binaryImage, binaryImage, erodeElement);
	erode(binaryImage, binaryImage, erodeElement);

	dilate(binaryImage, binaryImage, dilateElement);
	dilate(binaryImage, binaryImage, dilateElement);
}

uint32_t drawBoundingBoxes(Mat& destination, Mat& mask, Scalar color)
{
	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;
	uint32_t largest_contour_index = 0;
	uint32_t largest_area = 0;
	uint32_t i = 0;
	Rect bounding_rect;

	findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	for (; i < contours.size(); i++) {
		
		bounding_rect = boundingRect(contours[i]);
		rectangle(destination, bounding_rect, color, 2, 8, 0);
	}

	return i;
}

}
