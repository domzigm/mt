/*
 *
 * (c) domzigm 2016 - GPLv3
 * https://github.com/domzigm/mt
 * 
 */

#include "FigureDetection.h"
#include "HelperRoutines.h"

using namespace cv;

namespace mt
{

uint8_t locateFigure(figureDescription& desc, Mat& srcimage, Rect& rect)
{
	// Assume the srcimage is already in HSV color space
	Mat dstimage;
	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;
	uint32_t largest_contour_index = 0;
	uint32_t largest_area = 0;

	inRange(srcimage, desc.lowerThreshold, desc.upperThreshold, dstimage);
	morphOps(dstimage);

	findContours(dstimage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for (uint32_t i = 0; i < contours.size(); i++) {

		//  Find the area of contour
		double a = contourArea(contours[i], false);
		if (a > largest_area) {

			// Store the index of largest contour
			largest_contour_index = i;
		}
	}

	rect = boundingRect(contours[largest_contour_index]);
	return contours.size() > 0;
}

}
