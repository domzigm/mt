#include "BoardRectification.h"

using namespace cv;

namespace mt
{

bool BoardRectification::rectifyImage(Mat& cameraImage, Mat& rectifiedImage)
{
	bool ret = false;
	if (m_rectifyValid) {

		Point2f dstPts[4];

		// Image will be rectified to a specific static size
		dstPts[0] = Point2f(0, 0);
		dstPts[1] = Point2f((float)m_config.outputImageWidth, 0);
		dstPts[2] = Point2f((float)m_config.outputImageWidth, (float)m_config.outputImageHeight);
		dstPts[3] = Point2f(0, (float)m_config.outputImageHeight);

		Mat warp_mat = getPerspectiveTransform(m_rectifyCoords, dstPts);
		warpPerspective(cameraImage, rectifiedImage, warp_mat, rectifiedImage.size());
		rectifiedImage = rectifiedImage(Rect(0, 0, m_config.outputImageWidth, m_config.outputImageHeight));

		ret = true;
	}

	return ret;
}

bool BoardRectification::updateBoard(cv::Mat& cameraImage)
{
	int regionsFound = 0;
	cv::Mat grayImage;
	cv::cvtColor(cameraImage, grayImage, CV_RGB2GRAY);

	// Left upper corners
	const cv::Point2f coordEdges[MARKER_MAX] =
	{
		cv::Point2f(0.f,							0.f),
		cv::Point2f(1.f - m_config.sourceRoiWidth,	0.f),
		cv::Point2f(1.f - m_config.sourceRoiWidth,	1.f - m_config.sourceRoiHeight),
		cv::Point2f(0.f,							1.f - m_config.sourceRoiHeight)
	};

	for (int region = TOP_LEFT; region < MARKER_MAX; region++) {

		auto rect = Rect(
			(int)(coordEdges[region].x * grayImage.cols),		// X position
			(int)(coordEdges[region].y * grayImage.rows),		// Y position
			(int)(grayImage.cols * m_config.sourceRoiWidth),	// width
			(int)(grayImage.rows * m_config.sourceRoiHeight)	// height
		);

		// Set ROI on gray image
		Mat temp1 = grayImage(rect);
		Mat temp2;

		float scale_x = (float)temp1.cols / (float)m_config.analyzeRoiSize;
		float scale_y = (float)temp1.rows / (float)m_config.analyzeRoiSize;

		// Resize ROI to fixed size
		resize(temp1, temp2, cvSize(m_config.analyzeRoiSize, m_config.analyzeRoiSize), 0.0f, 0.0f, cv::INTER_CUBIC);

		std::vector<Point2i> corners;
		goodFeaturesToTrack(temp2, corners, 12, 0.4f, 5, noArray(), 7, true);

		// Calculate smallest distance from image edge to board corner
		double minDistance = m_config.analyzeRoiSize;
		for (auto corner : corners) {

			double distance = 0.f;

			switch (region)	{

			case TOP_LEFT:
				distance = abs(norm(corner - Point(0, 0)));
				break;
			case TOP_RIGHT:
				distance = abs(norm(corner - Point(m_config.analyzeRoiSize, 0)));
				break;
			case BOT_RIGHT:
				distance = abs(norm(corner - Point(m_config.analyzeRoiSize, m_config.analyzeRoiSize)));
				break;
			case BOT_LEFT:
				distance = abs(norm(corner - Point(0, m_config.analyzeRoiSize)));
				break;
			}

			// We can find up to 9 points for every ROI, find the one with the shortest distance to the edge
			if (distance < minDistance) {

				minDistance = distance;

				m_rectifyCoords[region].x = (corner.x * scale_x) + rect.x;
				m_rectifyCoords[region].y = (corner.y * scale_y) + rect.y;
			}
		}

		if (minDistance < m_config.analyzeRoiSize) {

			regionsFound++;
		}
	}

	m_rectifyValid = (MARKER_MAX == regionsFound);
	return m_rectifyValid;
}

}
