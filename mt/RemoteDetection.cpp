/*
 *
 * (c) domzigm 2016 - GPLv3
 * https://github.com/domzigm/mt
 * 
 */

#include "RemoteDetection.h"
#include "HelperRoutines.h"

using namespace cv;
using namespace std;

namespace mt
{

#define DEBUG 1

RemoteDetection::RemoteDetection(RemoteDetectionConfig& config) :
	m_config(config)
	, m_calibMarkerCount(2)
	, m_radiusTolerance(1.6f)
{
	// When using correction preprocessing, the SimpleBlobDetector is rather robust.
	// Therefore there is currently no need to alter the configuration.
	m_sbdParams.minDistBetweenBlobs = 20.0f;
	m_sbdParams.filterByInertia = true;
	m_sbdParams.filterByConvexity = true;
	m_sbdParams.filterByColor = false;
	m_sbdParams.filterByCircularity = true;
	m_sbdParams.filterByArea = true;
	m_sbdParams.minArea = (float)(m_sbdParams.minDistBetweenBlobs * 2 * CV_PI);
	m_sbdParams.maxArea = m_sbdParams.minArea * 20.f;
	m_sbdParams.minInertiaRatio = 0.3f;
	m_sbdParams.maxInertiaRatio = 1.0f;
	m_sbdParams.minConvexity = 0.3f;
	m_sbdParams.maxConvexity = 1.0f;
	m_sbdParams.minCircularity = 0.3f;
	m_sbdParams.maxCircularity = 1.0f;
}

void RemoteDetection::detectColoredButtons(const colorDescription& desc, const Mat& srcimage, vector<KeyPoint>& keypoints)
{
	Mat hsvImage;
	Mat hsvRoi;
	Mat filtered;

	// Resize, blur and color conversion
	resize(srcimage, m_scaledImage, Size(0, 0), m_config.scaleFactX, m_config.scaleFactY);
	//GaussianBlur(scaled, filtered, Size(7, 7), 0);
	cvtColor(m_scaledImage, hsvImage, CV_BGR2HSV);
	//hsvImage = srcimage;

	// Create the HSV roi
	inHsvRange(hsvImage, desc, hsvRoi);

	Mat k3 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	Mat k5 = getStructuringElement(MORPH_CROSS, Size(5, 5));
	Mat k7 = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));

	morphologyEx(hsvRoi, hsvRoi, MORPH_CLOSE, k5, Point(-1, -1), 4);
	morphologyEx(hsvRoi, hsvRoi, MORPH_OPEN, k3, Point(-1, -1), 1);

#ifdef DEBUG
	imshow("HSVROI", hsvRoi);
#endif

	// Convert to binary image and do morphology operations
	//Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	//morphologyEx(hsvRoi, hsvRoi, MORPH_DILATE, kernel);
	//morphologyEx(hsvRoi, hsvRoi, MORPH_DILATE, kernel);
	//morphologyEx(hsvRoi, hsvRoi, MORPH_ERODE, kernel);

	// Extract all blobs from the image
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(m_sbdParams);
	detector->detect(hsvRoi, keypoints);
}

void RemoteDetection::detectContrastButtons(const colorDescription& desc, const Mat& srcimage, vector<KeyPoint>& keypoints)
{
	Mat grayRoi;
	resize(srcimage, m_scaledImage, Size(0, 0), 1.f, 1.f);

	bwEdgeDetection(m_scaledImage, grayRoi, true, false, 100.f, 248.f);

	// Extract all blobs from the image
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(m_sbdParams);
	detector->detect(grayRoi, keypoints);

	imshow("grayroi", grayRoi);

}

RemoteStatus RemoteDetection::getRemoteStatus(const colorDescription& desc, const Mat& srcimage, uint8_t& buttonId)
{
	vector<cv::KeyPoint> markers;
	vector<cv::KeyPoint> buttons;
	vector<cv::KeyPoint> keypoints;
	detectColoredButtons(desc, srcimage, keypoints);
	//detectContrastButtons(desc, srcimage, keypoints);

	// There must be at least m_calibMarkerCount blobs and maximum m_calibMarkerCount + numButtons blobs
	if (keypoints.size() < m_calibMarkerCount) { // || keypoints.size() > m_config.buttonDistances.size() + m_calibMarkerCount) {
	//	return REMOTE_NOT_DETECTED;
	}

	// Find the smallest blob
	float minRadius = (float)(srcimage.rows * srcimage.rows + srcimage.cols * srcimage.cols);
	for (auto& point : keypoints) {

		minRadius = MIN(point.size, minRadius);
	}
	float threshRadius = minRadius * m_radiusTolerance;

	// Divide found blobs into remote markers and buttons
	for (auto& point : keypoints) {

		if (point.size > threshRadius) {

			buttons.push_back(point);
		}
		else {

			markers.push_back(point);
		}
	}

#ifdef DEBUG
	Mat dbgImage;
	m_scaledImage.copyTo(dbgImage);
	//cvtColor(m_scaledImage, dbgImage, CV_HSV2BGR);
	for (auto& point : markers) {

		circle(dbgImage, point.pt, (int)(point.size / 2), Scalar(0, 255, 0), 3, 8, 0);
	}
	for (auto& point : buttons) {

		circle(dbgImage, point.pt, (int)(point.size / 2), Scalar(0, 0, 255), 3, 8, 0);
	}
	imshow("RemoteDetection", dbgImage);
#endif

	// Check if we found the correct amount of calibration markers
	if (markers.size() != m_calibMarkerCount) {

		return REMOTE_NOT_DETECTED;
	}

	// C++ std guarantees the default value = 0
	std::vector<uint8_t> buttonFound(m_config.buttonDistances.size());

	// Calculate distances
  // Todo: We could have false positives in markers array
	auto normDistance = cv::norm(markers[0].pt - markers[1].pt);
	if (normDistance < 20.f || normDistance > 100.f) {
		return REMOTE_NOT_DETECTED;
	}

	for (auto& point : buttons) {

		auto distance = cv::norm(point.pt - markers[0].pt) / normDistance;

		uint8_t counter = 0;
		for (auto& dist : m_config.buttonDistances) {

			// If a button has been detected and it's in the predefined range
			if (distance > dist[0] && distance < dist[1]) {

				buttonFound[counter] = 1;
				break;
			}
			counter++;
		}
	}

	// If a button has not been detected, this is due to a finger being on the button :-)
	uint8_t counter = 1;
	for (auto& button : buttonFound) {

		if (button == 0) {

			buttonId = counter;
			return REMOTE_BUTTON_PRESSED;
		}
		counter++;
	}

	return REMOTE_NOT_PRESSED;
}

}
