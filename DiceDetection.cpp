/*
 *
 * (c) domzigm 2016 - GPLv3
 * https://github.com/domzigm/mt
 * 
 */
 
#include "DiceDetecion.h"

using namespace cv;

namespace mt
{

#define DEBUG

uint8_t DiceDetection::processImage(Mat& inputImage)
{
	Mat canny;
	Mat temp1;
	Mat temp2;
	DiceCandidate dc;
	
	std::vector<DiceCandidate> v_eyes;

	eyeCounter = 0;
	dices.clear();

	cv::cvtColor(inputImage, temp1, cv::COLOR_BGR2GRAY);
	cv::resize(temp1, temp2, Size(0, 0), m_config.scaleFactX, m_config.scaleFactY);
	Canny(temp2, canny, m_config.cannyTh1, m_config.cannyTh2, 3);
	
	for (int y = 0; y < canny.size().height; y += m_config.subSample)	{
    
		uchar* row = canny.ptr(y);
		for (int x = 0; x < canny.size().width; x += m_config.subSample) {
      
			if (row[x] <= 128) {
        
				int area = floodFill(canny, Point(x, y), CV_RGB(0, 0, 160));
				
				if (area > m_config.fillThLow && area < m_config.fillThHigh) {
          
#ifdef DEBUG
					floodFill(canny, Point(x, y), CV_RGB(0, 0, 255));
#endif
					dc.area = area;
					dc.pt.x = x;
					dc.pt.y = y;
					dc.id = eyeCounter++;
					v_eyes.push_back(dc);
				}
			}
		}
	}

#ifdef DEBUG
	imshow("DiceDetection", canny);
	//waitKey(0);
#endif

	// We've found all circles in the image

	uint8_t neighborFound = 0;
	std::vector<DiceCandidate> neighbors;
	
	while (v_eyes.size()) {

		DiceCandidate c;

		if (neighborFound) {

			neighborFound = false;
			c = neighbors.at(neighbors.size() - 1);
		}
		else {

			c = v_eyes[0];
			neighbors.push_back(c);
			auto it = v_eyes.erase(v_eyes.begin());
		}

		for (auto it = v_eyes.begin(); it != v_eyes.end(); ++it) {

			double distance = cv::norm(c.pt - it->pt);

			if (distance < m_config.maxEyeDistance) {

				neighbors.push_back(*it);
				it = v_eyes.erase(it);
				neighborFound = true;
				if (v_eyes.size() == 0) {
					// There are no further eyes, so no neighbor can be further found
					neighborFound = false;
					break;
				}
			}
		}

		if (neighborFound == false) {

			dices.push_back(neighbors);
			neighbors.clear();
		}
	}

	return (dices.size > 0);
}

void DiceDetection::getResult(uint32_t& dices, uint32_t& eyes)
{
	dices = (uint32_t)this->dices.size();
	eyes = this->eyeCounter;
}

void DiceDetection::drawBoxes(cv::Mat& image)
{
	if(this->dices.size() > 0)
	{
		for (auto d = this->dices.begin(); d != this->dices.end(); ++d) {

			int radius = 0;
			int minX = image.cols;
			int minY = image.rows;
			int maxX = 0;
			int maxY = 0;

			for (auto eyes = d->begin(); eyes != d->end(); ++eyes) {

				// A = PI * r²;
				radius = (int)sqrt(eyes->area / CV_PI) + 1u;
				maxX = MAX(maxX, eyes->pt.x);
				minX = MIN(minX, eyes->pt.x);
				maxY = MAX(maxY, eyes->pt.y);
				minY = MIN(minY, eyes->pt.y);
			}

			// This is the basic box
			Rect rc;
			rc.x = minX - radius;
			rc.y = minY;
			rc.width  = maxX - minX + 2 * radius;
			rc.height = maxY - minY + 2 * radius;
			
			// This is the maximum detection radius box
			Rect rc2;
			rc2.x = (int)(((rc.x + rc.width  / 2) - m_config.maxEyeDistance / 2) / m_config.scaleFactX);
			rc2.y = (int)(((rc.y + rc.height / 2) - m_config.maxEyeDistance / 2) / m_config.scaleFactY);
			rc2.width  = (int)(m_config.maxEyeDistance / m_config.scaleFactX);
			rc2.height = (int)(m_config.maxEyeDistance / m_config.scaleFactY);
			rectangle(image, rc2, cvScalar(0, 255, 0), 2);
		}
	}
}

}