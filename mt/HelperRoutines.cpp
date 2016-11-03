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

float slope(Vec4i points)
{
	return (float)(atan2(points[1] - points[3], points[2] - points[0]) * 180.0f / CV_PI);
}

void bwEdgeDetection(const Mat& srcImage, Mat& dstImage, bool useHls, bool useRGB, double lowerThres, double upperThresh)
{
}

void bwEdgeDetection2(const Mat& srcImage, Mat& dstImage, bool useHls, bool useRGB, double lowerThres, double upperThresh)
{
	// Do a canny edge detection
	// Check if the edges are pointing to a certain threshold value in the image
	// Check on how to remove unwanted noise (e.g. downscale + upscale)

	Mat mlow, mhigh, hls, split_hls[3];
	//Mat morph3 = getStructuringElement(MORPH_RECT, Size(3, 3));
	//Mat morph5 = getStructuringElement(MORPH_RECT, Size(5, 5));
	Mat morph3 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	Mat morph5 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

	if (srcImage.channels() == 1) {
		// Source image has only one channel, assume it's gray
		srcImage.copyTo(split_hls[1]);
	}
	else {
		// Source image has more than one channel
		if (useHls) {
			// Convert RGB to HLS color
			useRGB ? cvtColor(srcImage, hls, CV_RGB2HLS) : cvtColor(srcImage, hls, CV_BGR2HLS);
			split(hls, split_hls);
		}
		else {
			// Convert RGB to GRAY
			useRGB ? cvtColor(srcImage, split_hls[1], CV_RGB2GRAY) : cvtColor(srcImage, split_hls[1], CV_BGR2GRAY);
		}
	}

	GaussianBlur(split_hls[1], split_hls[1], Size(5, 5), 0);
	equalizeHist(split_hls[1], hls);

	//imshow("bwEdges", hls);

	Mat canny;
	Canny(hls, canny, 400, 400, 3, true);
	morphologyEx(canny, canny, MORPH_CLOSE, morph3, Point(-1, -1), 2);

	threshold(hls, mlow, lowerThres, 255, CV_THRESH_BINARY_INV);
	threshold(hls, mhigh, upperThresh, 255, CV_THRESH_BINARY);
#if 0
	for (int y = 0; y < mlow.size().height; y += 1) {

		uchar* row = mlow.ptr(y);
		for (int x = 0; x < mlow.size().width; x += 1) {

			if (row[x] == 255) {

				int area = floodFill(mlow, Point(x, y), CV_RGB(1, 1, 1));
				if (area < 1000) {
					int area = floodFill(mlow, Point(x, y), CV_RGB(254, 254, 254));
				}
				else {
					int area = floodFill(mlow, Point(x, y), CV_RGB(0, 0, 0));
				}
			}
		}
	}

	for (int y = 0; y < mhigh.size().height; y += 1) {

		uchar* row = mhigh.ptr(y);
		for (int x = 0; x < mhigh.size().width; x += 1) {

			if (row[x] > 1 && row[x] != 254) {

				int area = floodFill(mhigh, Point(x, y), CV_RGB(1, 1, 1));
				if (area < 100) {
					int area = floodFill(mhigh, Point(x, y), CV_RGB(0, 0, 0));
				}
				else {
					int area = floodFill(mhigh, Point(x, y), CV_RGB(254, 254, 254));
				}
			}
		}
	}
#endif
	morphologyEx(mhigh, mhigh, MORPH_DILATE, morph3, Point(-1, -1), 3);
	morphologyEx(mlow, mlow, MORPH_DILATE, morph3, Point(-1, -1), 3);

	//mlow = ~mlow;
	imshow("low", mlow);
	imshow("high", mhigh);
	imshow("canny", canny);

	//mhigh = ~mhigh;
	//mlow = ~mlow;

	//morphologyEx(mlow, mlow, MORPH_DILATE, morph5);
	//morphologyEx(mlow, mlow, MORPH_DILATE, morph5);
	//morphologyEx(mlow, mlow, MORPH_DILATE, morph5);
	//morphologyEx(mlow, mlow, MORPH_DILATE, morph5);
	//morphologyEx(mhigh, mhigh, MORPH_DILATE, morph3);
	//dstImage = mlow & canny;// mhigh;
	dstImage = canny & mhigh;

	morphologyEx(dstImage, dstImage, MORPH_CLOSE, morph3, Point(-1, -1), 2);

	for (int y = 0; y < dstImage.size().height; y += 1) {

		uchar* row = dstImage.ptr(y);
		for (int x = 0; x < dstImage.size().width; x += 1) {

			if (row[x] == 0) {

				int area = floodFill(dstImage, Point(x, y), CV_RGB(1, 1, 1));
				if (area > 1 && area < 1000) {
					floodFill(dstImage, Point(x, y), CV_RGB(255, 255, 255));
				}
			}
		}
	}

	morphologyEx(dstImage, dstImage, MORPH_ERODE, morph5, Point(-1, -1), 4);

	//morphologyEx(dstImage, dstImage, MORPH_DILATE, morph3, Point(-1, -1), 2);
	//morphologyEx(dstImage, dstImage, MORPH_CLOSE, morph3, Point(-1, -1), 5);
}

void inHsvRange(const Mat& srcImage, const hsvPlanes& desc, Mat& dstImage)
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

void inHsvRange(const Mat& srcImage, const colorDescription& desc, cv::Mat& dstImage)
{
	inHsvRange(srcImage, desc.lowerThreshold, desc.upperThreshold, dstImage);
}

void inHsvRange(const Mat& srcImage, Scalar lowThres, Scalar highThres, Mat& dstImage)
{
	// If low threshold is bigger than high threshold, this means wrap around for red
	if (lowThres[0] > highThres[0]) {

		cv::Mat first;
		cv::Mat second;

		auto temp = highThres[0];
		highThres[0] = 179.f;
		inRange(srcImage, lowThres, highThres, first);

		highThres[0] = temp;
		lowThres[0] = 0.f;
		inRange(srcImage, lowThres, highThres, second);

		dstImage = first | second;
	}
	else {
		inRange(srcImage, lowThres, highThres, dstImage);
	}
}

//! Convert absolute (x,y) rect values into relative values
void rectToRelative(const Mat& mapping, const Rect& input_rect, Rect2f& output_rect)
{
	output_rect.x = (float)(input_rect.x / mapping.cols);
	output_rect.y = (float)(input_rect.y / mapping.rows);
	output_rect.width = (float)(input_rect.width / mapping.cols);
	output_rect.height = (float)(input_rect.height / mapping.rows);
}

//! Convert relative rect values into absolute (x,y) values
void rectToAbsolute(const Mat& mapping, const Rect2f& input_rect, Rect& output_rect)
{
	output_rect.x = (int)(input_rect.x * mapping.cols);
	output_rect.y = (int)(input_rect.y * mapping.rows);
	output_rect.width = (int)(input_rect.width * mapping.cols);
	output_rect.height = (int)(input_rect.height * mapping.rows);
}

//! This will also include a bit of the surrounding area
void morphOps(Mat& binaryImage)
{
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(7, 7));

	erode(binaryImage, binaryImage, erodeElement, Point(-1, -1), 2);
	dilate(binaryImage, binaryImage, dilateElement, Point(-1, -1), 2);
}

//! Draw bounding boxes on an image based on a binary mask
uint32_t drawBoundingBoxes(Mat& destination, Mat& mask, Scalar color)
{
	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;
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
