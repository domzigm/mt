/*
 *
 * (c) domzigm 2016 - GPLv3
 * https://github.com/domzigm/mt
 * 
 */

#include "RemoteDetection.h"

using namespace cv;
using namespace std;

namespace mt
{

#define DEBUG

RemoteDetection::RemoteDetection(RemoteDetectionConfig& config) :
	m_config(config)
	, m_calibMarkerCount(2)
	, m_radiusTolerance(1.2f)
{
	// When using correction preprocessing, the SimpleBlobDetector is rather robust.
	// Therefore there is currently no need to alter the configuration.
	m_sbdParams.minDistBetweenBlobs = 50.0f;
	m_sbdParams.filterByInertia = true;
	m_sbdParams.filterByConvexity = true;
	m_sbdParams.filterByColor = false;
	m_sbdParams.filterByCircularity = true;
	m_sbdParams.filterByArea = true;
	m_sbdParams.minArea = (float)(m_sbdParams.minDistBetweenBlobs * 2 * CV_PI);
	m_sbdParams.maxArea = m_sbdParams.minArea * 20.f;
	m_sbdParams.minInertiaRatio = 0.2f;
	m_sbdParams.maxInertiaRatio = 1.0f;
	m_sbdParams.minConvexity = 0.2f;
	m_sbdParams.maxConvexity = 1.0f;
	m_sbdParams.minCircularity = 0.2f;
	m_sbdParams.maxCircularity = 1.0f;
}

RemoteStatus RemoteDetection::getRemoteStatus(hsvPlanes& desc, cv::Mat& srcimage, uint8_t& buttonId)
{
	Mat scaled;
	Mat grayImage;
	Mat hsvImage;
	Mat hsvRoi;
	Mat filtered;

	// Resize the image
	resize(srcimage, scaled, Size(0, 0), m_config.scaleFactX, m_config.scaleFactY);
	GaussianBlur(scaled, filtered, Size(7, 7), 3, 3);

	cvtColor(filtered, hsvImage, CV_BGR2HSV);
	cvtColor(filtered, grayImage, CV_BGR2GRAY);

	inHsvRange(hsvImage, desc, hsvRoi);

#ifdef DEBUG
	imshow("HSVROI", hsvRoi);
#endif

	// Convert to binary image and do morphology operations
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(hsvRoi, hsvRoi, MORPH_DILATE, kernel);
	morphologyEx(hsvRoi, hsvRoi, MORPH_DILATE, kernel);
	morphologyEx(hsvRoi, hsvRoi, MORPH_ERODE,  kernel);

	// Extract all blobs from the image
	vector<cv::KeyPoint> keypoints;
	vector<cv::KeyPoint> markers;
	vector<cv::KeyPoint> buttons;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(m_sbdParams);
	detector->detect(hsvRoi, keypoints);

	// There must be at least m_calibMarkerCount blobs and maximum m_calibMarkerCount + numButtons blobs
	if (keypoints.size() < m_calibMarkerCount || keypoints.size() > m_config.buttonDistances.size() + m_calibMarkerCount) {
		return REMOTE_NOT_DETECTED;
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
	for (auto& point : markers) {

		circle(grayImage, point.pt, (int)(point.size / 2), Scalar(0, 0, 0), 3, 8, 0);
	}

	for (auto& point : buttons) {

		circle(grayImage, point.pt, (int)(point.size / 2), Scalar(255, 255, 255), 3, 8, 0);
	}

	imshow("RemoteDetection", grayImage);
#endif

	if (markers.size() != 2) {

		return REMOTE_NOT_DETECTED;
	}

	std::vector<uint8_t> buttonPressed(m_config.buttonDistances.size());
	for (auto& button : buttonPressed) {

		button = 1;
	}

	// Calculate distances
	auto normDistance = cv::norm(markers[0].pt - markers[1].pt);
	for (auto& point : buttons) {

		auto distance = cv::norm(point.pt - markers[0].pt) / normDistance;

		uint8_t counter = 0;
		for (auto& dist : m_config.buttonDistances) {

			if (distance > dist[0] && distance < dist[1]) {

				buttonPressed[counter] = 0;
				break;
			}
			counter++;
		}
	}

	uint8_t counter = 1;
	for (auto& button : buttonPressed) {

		if (button > 0) {

			buttonId = counter;
			return REMOTE_BUTTON_PRESSED;
		}
		counter++;
	}

	return REMOTE_NOT_PRESSED;
}

}
