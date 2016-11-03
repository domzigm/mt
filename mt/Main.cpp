#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>

#include "BoardRectification.h"
#include "DiceDetection.h"
#include "HelperRoutines.h"
#include "FigureDetection.h"
#include "RemoteDetection.h"

using namespace mt;
using namespace cv;
using namespace std;

const std::string imagePath = "C:\\temp\\_spielfeld.jpg";

void adjustBrightnessContrast(cv::Mat& inputImage, cv::Mat& outputImage, double alpha, int beta)
{
	outputImage = cv::Mat::zeros(inputImage.size(), inputImage.type());

	for (int y = 0; y < inputImage.rows; y++)
	{
		for (int x = 0; x < inputImage.cols; x++)
		{
			int c = 0;
			//for (int c = 0; c < 3; c++)
			{
				
				if(y < inputImage.rows / 2)
				outputImage.at<uchar>(y, x) = cv::saturate_cast<uchar>(alpha*(inputImage.at<uchar>(y, x)) + beta);
				else
					outputImage.at<uchar>(y, x) = cv::saturate_cast<uchar>((inputImage.at<uchar>(y, x)));
			}
		}
	}
}
#if 0
bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2,
	cv::Point2f &r)
{
	using namespace cv;
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}
#endif

bool intersection1(cv::Vec4i& line1, cv::Vec4i& line2, cv::Point2i &xing)
{
	using namespace cv;
	Point2f o1 = Point2i(line1[0], line1[1]);
	Point2f p1 = Point2i(line1[1], line1[2]);
	Point2f o2 = Point2i(line2[0], line2[1]);
	Point2f p2 = Point2i(line2[1], line2[2]);

	Point2f ret;

//	auto val = intersection(o1, p2, o2, p2, ret);

	xing = ret;

	return 1;

	/*
	Point2i x  = Point2i(line2[0] - line2[1]) - Point2i(line1[0], line1[1]);
	Point2i d1 = Point2i(line1[2], line1[3]) - Point2i(line1[0], line1[1]);
	Point2i d2 = Point2i(line2[2], line2[3]) - Point2i(line2[0], line2[1]);

	int cross = d1.x * d2.y - d1.y * d2.x;
	if (cross < 0)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	xing = Point2i(line1[0], line1[1]) + d1 * t1;
	return true;
	*/
	/*
	cv::Point2i x = o2 - o1;
	cv::Point2i d1 = p1 - o1;
	cv::Point2i d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < 1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
	*/
}

using namespace cv;

double cross(Point v1, Point v2) {
	return v1.x*v2.y - v1.y*v2.x;
}

bool getIntersectionPoint(Point a1, Point a2, Point b1, Point b2, Point & intPnt) {
	Point p = a1;
	Point q = b1;
	Point r(a2 - a1);
	Point s(b2 - b1);

	if (cross(r, s) == 0) { return false; }

	double t = cross(q - p, s) / cross(r, s);

	intPnt = p + t*r;

	/*
	if (intPnt.x > a1.x && intPnt.x > a2.x && intPnt.x > b1.x && intPnt.x > b2.x)
		return false;
	if (intPnt.y > a1.y && intPnt.y > a2.y && intPnt.y > b1.y && intPnt.y > b2.y)
		return false;
	*/

	return true;
}


using namespace cv;


#if 0
bool BoardRectification::updateBoard(cv::Mat& cameraImage)
{
	cv::Mat grayImage;
	cv::cvtColor(cameraImage, grayImage, CV_RGB2GRAY);

	for (int region = TOP_LEFT; region < MARKER_MAX; region++) {

		auto rect = Rect(
			coordEdges[region].x * grayImage.cols,	// X position
			coordEdges[region].y * grayImage.rows,	// Y position
			grayImage.cols * roi_width,				// width
			grayImage.rows * roi_height				// height
		);

		Mat temp1 = grayImage(rect);
		Mat temp2;
		Mat canny;
		Mat harris;
		Mat harris_norm;

		float scale_x = (float)temp1.cols / (float)fixedRoiSize;
		float scale_y = (float)temp1.rows / (float)fixedRoiSize;

		//temp2 = temp1;
		//bilateralFilter(temp1, temp2, 5.0f, 50.f, 50.f);
		resize(temp1, temp2, cvSize(fixedRoiSize, fixedRoiSize), 0.0f, 0.0f, cv::INTER_CUBIC);

		std::vector<Point2i> corners;
		goodFeaturesToTrack(temp2, corners, 12, 0.4f, 5, noArray(), 7, true);

		for (auto corner : corners)
		{
			//corner.x += coordEdges[region].x;
			//corner.y += coordEdges[region].y;
			line(temp2, corner, corner, cv::Scalar(255, 0, 255), 3, CV_AA);

			corner.x *= scale_x;
			corner.y *= scale_y;
			corner.x += rect.x;
			corner.y += rect.y;

			line(cameraImage, corner, corner, cv::Scalar(255, 0, 255), 3, CV_AA);
		}
		imshow("test", temp2);
		waitKey(0);

		//equalizeHist(temp1, canny);

		//Canny(temp1, canny, canny_thres, canny_thres * 2.f);

		//resize(temp1, temp2, cvSize(fixedRoiSize, fixedRoiSize), 0.0f, 0.0f, cv::INTER_CUBIC);
//		cornerHarris(temp2, harris, 3, 7, 0.03f);
		

//		normalize(harris, temp1, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
//		convertScaleAbs(temp1, temp2);

//		threshold(temp2, canny, 170, 255, THRESH_TOZERO);

//		imshow("corners", canny);
//		waitKey(0);
	}
		/*
    Canny(temp1, canny, canny_thres, canny_thres * 2.f);
		
		cv::Mat kernel(3, 3, CV_8U);
		kernel.data[0] = 0;
		kernel.data[1] = 1;
		kernel.data[2] = 0;
		kernel.data[3] = 1;
		kernel.data[4] = 1;
		kernel.data[5] = 1;
		kernel.data[6] = 0;
		kernel.data[7] = 1;
		kernel.data[8] = 0;

		cv::dilate(canny, canny, kernel);
		cv::erode(canny, canny, cv::Mat());

		std::vector<cv::Vec4i> lines;
		HoughLinesP(canny, lines, 1, CV_PI / 180, 50, 30, 15);

		for (size_t i = 0; i < lines.size(); i++)
		{
			cv::Vec4i l = lines[i];

			Point a = Point(l[0] * scale_x, l[1] * scale_y);
			Point b = Point(l[2] * scale_x, l[3] * scale_y);

			a.x += rect.x;
			a.y += rect.y;
			b.x += rect.x;
			b.y += rect.y;

			cv::line(cameraImage, a, b, cv::Scalar(255, 0, 0), 2, CV_AA);
		}

		Point2i avg_edge;
		int rounds = 0;

		for (int i = 0; i < lines.size() - 1; i++)
		{
			cv::Vec4i ol = lines[i];

			for (int j = i + 1; j < lines.size(); j++)
			{
				cv::Vec4i il = lines[j];

				cv::Point2i pt;

				// Change from point to vec4i
				if (getIntersectionPoint(
					Point(ol[0], ol[1]),
					Point(ol[2], ol[3]),
					Point(il[0], il[1]),
					Point(il[2], il[3]),
					pt
					)
				) {

					pt.x *= scale_x;
					pt.y *= scale_y;
					pt.x += rect.x;
					pt.y += rect.y;

					if (pt.x > rect.x &&
						pt.x < rect.x + (rect.width) &&
						pt.y > rect.y &&
						pt.y < rect.y + (rect.height))

					{
						cv::line(cameraImage, pt, pt, cv::Scalar(255, 0, 255), 10, CV_AA);

						avg_edge += pt;
						rounds++;
					}
				}
			}
		}

		if (rounds > 0)
		{
			// Requires outlier filter :-(
			avg_edge /= rounds;
			rectifyCoords[region] = avg_edge;
			cv::line(cameraImage, avg_edge, avg_edge, cv::Scalar(0, 255, 0), 10, CV_AA);
		}

		boardEdges[region] = canny;
		
	}

	rectifyValid = true; // Check all edges before setting to true

//	imshow("TOP_LEFT", boardEdges[TOP_LEFT]);
//	imshow("TOP_RIGHT", boardEdges[TOP_RIGHT]);
//	imshow("BOT_RIGHT", boardEdges[BOT_RIGHT]);
//	imshow("BOT_LEFT", boardEdges[BOT_LEFT]);
	
	Mat mainImage;
	resize(cameraImage, mainImage, cvSize(cameraImage.cols / 2, cameraImage.rows / 2));
	imshow("MAIN", mainImage);
	waitKey(0);
*/
//imshow("MAIN", cameraImage);
//waitKey(0);
	return true;
}
#endif

Scalar LowG(50, 85, 60);
Scalar HighG(70, 175, 140);

Scalar LowR(0, 165, 150);
Scalar HighR(5, 220, 235);

Scalar LowY(15, 145, 155);
Scalar HighY(30, 255, 255);

Scalar LowB(105, 90, 100);
Scalar HighB(120, 180, 250);

void drawContur(Mat& destination, Mat& mask, Scalar color, Point& coords)
{
	int largest_contour_index = 0;
	int largest_area = 0;
	Rect bounding_rect;
	std::vector<std::vector<Point>> contours; // Vector for storing contour
	std::vector<Vec4i> hierarchy;
	findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	for (int i = 0; i< contours.size(); i++)
	{
		// Find the area of contour
		double a = contourArea(contours[i], false);
		//if (a>largest_area) {
			// Store the index of largest contour
			largest_contour_index = i;
			// Find the bounding rectangle for biggest contour
			bounding_rect = boundingRect(contours[i]);
			rectangle(destination, bounding_rect, color, 2, 8, 0);
		//}
	}

	//rectangle(destination, bounding_rect, color, 2, 8, 0);
}

void doDice()
{
	//Mat imin = imread("C:\\temp\\dice\\dice1.jpg");
	Mat imin = imread("C:\\temp\\dice\\20160713_065204.jpg");
	Mat dice;

	cv::resize(imin, dice, Size(0, 0), 0.5f, 0.5f);

	Mat filtered;
	Mat resized;
	Mat temp;
	Mat temp2;
	Mat canny;

	DiceDetectionConfig ddc;
	ddc.cannyTh1 = 250;
	ddc.cannyTh2 = 500;
	ddc.fillThLow = 150;
	ddc.fillThHigh = 250;
	ddc.maxEyeDistance = 60.f;
	ddc.scaleFactX = 0.99f;
	ddc.scaleFactY = 0.99f;
	ddc.subSample = 2;

	DiceDetection dc(ddc);

	uint32_t dices = 0;
	uint32_t eyes = 0;
	dc.processImage(dice);
	dc.getResult(dices, eyes);
	dc.drawBoxes(dice);

	printf("Dices: %d, Eyes: %d\r\n", dices, eyes);

	imshow("DICE", dice);
	waitKey(0);

	//exit(0);

#if 0
	cv::cvtColor(dice, temp, cv::COLOR_BGR2GRAY);
	cv::resize(temp, resized, Size(0, 0), 0.5f, 0.5f);

	namedWindow("canny");   

	int key = 0;

	int th1 = 250;
	int th2 = 500;
	int aperture = 3;
	int eyes = 0;

	createTrackbar("th1", "canny", &th1, 1000);
	createTrackbar("th2", "canny", &th2, 1000);
	createTrackbar("EYES", "canny", &eyes, 30);

	std::vector<candidates> cd;
	std::vector<candidates> neighbors;
	std::vector<candidates> _dice;
	std::vector<candidates> _dicesLeft;
	std::vector< std::vector<candidates> > _dices;
	candidates c;

	while (key != 27)
	{
		int la = aperture;
		if (la % 2 == 0)
			la += 1;
		Canny(resized, temp, (double)th1, (double)th2, la);
		
	//	dilate(temp, canny, Mat());
	//	erode(canny, canny, Mat());
		canny = temp;
		eyes = 0;
		
		int ctr = 0;
		int num = 0;
		for (int y = 0; y < canny.size().height; y++)
		{
			uchar* row = canny.ptr(y);
			for (int x = 0; x < canny.size().width; x++)
			{
				if (row[x] <= 128)
				{
					int area = floodFill(canny, Point(x, y), CV_RGB(0, 0, 160));
					
					//imshow("canny", canny);
					//key = waitKey(0);
					
					if (area > 150 && area < 250)
					{
						num++;
						c.area = floodFill(canny, Point(x, y), CV_RGB(0, 0, 255));
						c.pt.x = x;
						c.pt.y = y;
						c.id = ctr++;
						cd.push_back(c);
					}
					/*
					if (area < 100)
					{
						floodFill(canny, Point(x, y), CV_RGB(0, 0, 0));
					}
					*/
				}
			}
		}
		setTrackbarPos("EYES", "canny", num);
		/*dilate(canny, canny, Mat());
		dilate(canny, canny, Mat());
		dilate(canny, canny, Mat());
		dilate(canny, canny, Mat());*/
		imshow("canny", canny);
		
		//key = waitKey(0);
		uint8_t neighborFound = 0;

		while (cd.size()) {

			candidates c;

			if (neighborFound) {
				neighborFound = false;
				c = neighbors.at(neighbors.size() - 1);
			}
			else {
				c = cd[0];
				neighbors.push_back(c);
				cd.erase(cd.begin());
			}

			for ( auto it = cd.begin(); it != cd.end(); ++it)
			{
				double distance = cv::norm(c.pt - it->pt);
				//printf("Distance: %f\r\n", distance);

				if (distance < 40.f) {
					neighbors.push_back(*it);
					it = cd.erase(it);
					neighborFound = true;
					//break;
				}
			}

			
			if (neighborFound == false) {
				_dices.push_back(neighbors);
				printf("Found dice with %u eyes\r\n", neighbors.size());
				neighbors.clear();
			}


				//printf("-----\r\n");
				
			//}

			//printf("==========\r\n");
			//waitKey(0);
		}

		for (auto d = _dices.begin(); d != _dices.end(); ++d)
		{
			int minX = 1280, minY = 1280;
			int maxX = 0, maxY = 0;
			int adjust = 0;
			for (auto eyes = d->begin(); eyes != d->end(); ++eyes)
			{
				//A = pi * r²;
				adjust = sqrt(eyes->area / CV_PI) + 1;
				maxX = MAX(maxX, eyes->pt.x);
				minX = MIN(minX, eyes->pt.x);
				maxY = MAX(maxY, eyes->pt.y);
				minY = MIN(minY, eyes->pt.y);
			}

			Rect rc;
			rc.x = minX - adjust;
			rc.y = minY;
			rc.width = maxX - minX + 2*adjust;
			rc.height = maxY - minY + 2*adjust;
			//rectangle(canny, rc, cvScalar(0, 0, 0), 2);

			auto boxs = 70;
			Rect rc2;
			rc2.x = (rc.x + rc.width / 2) - boxs / 2;
			rc2.y = (rc.y + rc.height / 2) - boxs / 2;
			rc2.width = boxs;
			rc2.height = boxs;
			rectangle(canny, rc2, cvScalar(0, 0, 0), 2);

		}
		imshow("canny", canny);
		key = waitKey(0);
	}

#endif


	//exit(0);
}

void scale(Point2f& pt, Mat img)
{
	pt.x /= img.cols;
	pt.y /= img.rows;
}

void doRemote()
{
	Mat remote = imread("C:\\temp\\20160815_145546.jpg");
	//Mat remote = imread("C:\\temp\\20160815_145554.jpg");
	Mat resized;
	Mat canny;
	Mat filtered;
	Mat gray;

	cv::resize(remote, resized, Size(0, 0), 0.5f, 0.5f);

	cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);
	
	GaussianBlur(resized, filtered, Size(7, 7), 3, 3);
	Canny(filtered, canny, 15, 25);
	//cv::imshow("remote", canny);
	//if (27 == cv::waitKey(0))
	//	exit(0);

	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 50.0f;
	params.filterByInertia = true;
	params.filterByConvexity = true;
	params.filterByColor = false;
	params.filterByCircularity = true;
	params.filterByArea = true;
	params.minArea = 100.0f;
	params.maxArea = 7500.0f;
	params.minInertiaRatio = 0.35f;
	params.maxInertiaRatio = 1.0f;
	params.maxConvexity = 1.0f;
	params.minConvexity = 0.35f;
	params.minCircularity = 0.8f;
	params.maxCircularity = 1.0f;
	
	// ... any other params you don't want default value

	// set up and create the detector using the parameters
	//cv::SimpleBlobDetector blob_detector(params);
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	// detect!
	vector<cv::KeyPoint> keypoints;
	detector->detect(canny, keypoints);

	for (auto& point : keypoints)
	{
		circle(resized, point.pt, 10, Scalar(0, 0, 255), 3, 8, 0);
	}

	auto start = keypoints[0].pt;
	scale(start, resized);
	auto prev = start;

	for (int i = 1; i < keypoints.size(); i++) {
		Point2f temp = keypoints[i].pt;
		scale(temp, resized);
		auto distStart = cv::norm(start - temp);
		auto distPrev = cv::norm(prev - temp);
		prev = temp;

		printf("distStart: %f  -  distPrev: %f\r\n", distStart, distPrev);
	}
	
	

	cv::imshow("remote", resized);
	if (27 == cv::waitKey(0))
		exit(0);



	vector<Vec3f> circles;

	cv::namedWindow("remote");
	
	int mindist = 60;
	int param1 = 230;
	int param2 = 50;
	int minRadius = 10;
	int maxRadius = 140;
	
	cv::createTrackbar("mindist", "remote", &mindist, 1000);
	cv::createTrackbar("param1", "remote", &param1, 1000);
	cv::createTrackbar("param2", "remote", &param2, 1000);
	cv::createTrackbar("minRadius", "remote", &minRadius, 1000);
	cv::createTrackbar("maxRadius", "remote", &maxRadius, 1000);

	while (1)
	{

		cv::resize(remote, resized, Size(0, 0), 0.5f, 0.5f);

		cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

		GaussianBlur(gray, filtered, Size(7, 7), 3, 3);
		adaptiveThreshold(filtered, canny, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 7, 2);

		//erode(canny, canny, Mat());
		//erode(canny, canny, Mat());
		//erode(canny, canny, Mat());
		
		erode(canny, canny, Mat());
		dilate(canny, canny, Mat());
		dilate(canny, canny, Mat());
		
		
		//dilate(canny, canny, Mat());

		//cv::bilateralFilter(gray, filtered, 5.0f, 50.f, 50.f);
		//cv::Canny(gray, filtered, 50, 75, 3);

		HoughCircles(canny, circles, CV_HOUGH_GRADIENT, 2, mindist, param1, param2, minRadius, maxRadius);

		for (size_t i = 0; i < circles.size(); i++)
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// draw the circle center
			circle(resized, center, 3, Scalar(0, 255, 0), -1, 8, 0);
			// draw the circle outline
			circle(resized, center, radius, Scalar(0, 0, 255), 3, 8, 0);
		}

		cv::imshow("remote", canny);
		if (27 == cv::waitKey(250))
			exit(0);
	}

	cv::Canny(filtered, canny, 50, 75, 3);
	
	std::vector<CvPoint>points;

	for (int y = 0; y < canny.size().height; y++)
	{
		uchar* row = canny.ptr(y);
		for (int x = 0; x < canny.size().width; x++)
		{
			if (row[x] <= 128)
			{
				int area = floodFill(canny, Point(x, y), CV_RGB(0, 0, 160));

				if (area > 10 && area < 50)
				{
					points.push_back(cvPoint(x, y));
					floodFill(canny, Point(x, y), CV_RGB(0, 0, 255));
					cv::line(canny, Point(x, y), Point(x, y), cv::Scalar(0, 0, 0), 3, CV_AA);
				}
			}
		}
	}

	double angle = atan2(points[0].y - points[1].y, points[0].x - points[1].x) * 180 / CV_PI;

	printf("Angle: %f\r\n", 180 + angle);

	cv::imshow("remote", canny);
	cv::waitKey(0);

	exit(0);
}

// Red: 145-179, 155-255, 90-155
// Blue: 100-130, 95-195, 105-165
// Yellow: 25-65, 100-200, 160-210
// Green: 60-105, 150-255, 20-120

void doRemote2()
{
	//Mat inputImg = imread("C:\\temp\\20160824_172043.jpg");
	Mat inputImg = imread("C:\\temp\\20160824_172056.jpg");
	/*
	///////////////////////////////////
	RemoteDetectionConfig rdc;
	rdc.scaleFactX = 0.75f;
	rdc.scaleFactY = 0.75f;
	rdc.buttonDistances.push_back(Vec2f(1.38f, 1.58f));
	rdc.buttonDistances.push_back(Vec2f(2.40f, 2.60f));
	rdc.buttonDistances.push_back(Vec2f(3.44f, 3.64f));
	rdc.buttonDistances.push_back(Vec2f(4.48f, 4.68f));
	
	remoteDescription rds;
	rds.lowerThreshold = Scalar(0, 200, 105);
	rds.upperThreshold = Scalar(179, 255, 170);

	uint8_t buttonId = 0;
	RemoteDetection rd(rdc);
	rd.getRemoteStatus(rds, inputImg, buttonId);

	cv::waitKey(0);
	return;
	*/
	///////////////////////////////////

	Mat scaled;
	Mat hsvImage;
	Mat grayImage;
	Mat filtered;
	Mat canny;
	Mat blobs;
	Mat output;
	cv::resize(inputImg, scaled, Size(0, 0), 0.5f, 0.5f);

	int iLowH = 0, iHighH = 19, iLowS = 200, iHighS = 255, iLowV = 105, iHighV = 170;
	int iLowH2 = 160, iHighH2 = 179;

	cv::cvtColor(scaled, hsvImage, CV_BGR2HSV);
	cv::cvtColor(scaled, grayImage, CV_BGR2GRAY);

//	cv::GaussianBlur(grayImage, filtered, Size(5, 5), 3, 3);
//	cv::Canny(filtered, canny, 15, 25);

	cv::imshow("output", scaled);
//	cv::imshow("canny", canny);

	cv::createTrackbar("LowH", "output", &iLowH, 179); //Hue (0 - 179)
	cv::createTrackbar("HighH", "output", &iHighH, 179);
	cv::createTrackbar("LowH2", "output", &iLowH2, 179); //Hue (0 - 179)
	cv::createTrackbar("HighH2", "output", &iHighH2, 179);

	cv::createTrackbar("LowS", "output", &iLowS, 255); //Saturation (0 - 255)
	cv::createTrackbar("HighS", "output", &iHighS, 255);

	cv::createTrackbar("LowV", "output", &iLowV, 255);//Value (0 - 255)
	cv::createTrackbar("HighV", "output", &iHighV, 255);

	Mat hsvRoi;

	cv::Vec2f distances[4];

	distances[0] = Vec2f(1.38f, 1.58f);
	distances[1] = Vec2f(2.40f, 2.60f);
	distances[2] = Vec2f(3.44f, 3.64f);
	distances[3] = Vec2f(4.48f, 4.68f);

	uint8_t buttonPressed[4] = { 0 };

	while (1)
	{
		blobs = Scalar(0, 0, 0);
		
		/*
		inRange(hsvImage,
			Scalar(iLowH, iLowS, iLowV),
			Scalar(iHighH, iHighS, iHighV),
			hsvRoi);
			*/

		/*
		hsvPlanes desc;
		desc.loThreshold_Left = Scalar(iLowH, iLowS, iLowV);
		desc.hiThreshold_Left = Scalar(iHighH, iHighS, iHighV);
		desc.loThreshold_Right = Scalar(iLowH2, iLowS, iLowV);
		desc.hiThreshold_Right = Scalar(iHighH2, iHighS, iHighV);
		inHsvRange(hsvImage, desc, hsvRoi);
		*/

		inHsvRange(hsvImage,
			Scalar(iLowH, iLowS, iLowV),
			Scalar(iHighH, iHighS, iHighV),
			hsvRoi);
		
		//morphOps(hsvRoi);
		//dilate(hsvRoi, hsvRoi, Mat());
		//dilate(hsvRoi, hsvRoi, Mat());
		//erode(hsvRoi, hsvRoi, Mat());

		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
		//threshold(blobs, blobs, 30, 255, THRESH_BINARY);
		morphologyEx(hsvRoi, hsvRoi, MORPH_DILATE, kernel);
		morphologyEx(hsvRoi, hsvRoi, MORPH_DILATE, kernel);
		morphologyEx(hsvRoi, hsvRoi, MORPH_ERODE, kernel);
		grayImage.copyTo(blobs, hsvRoi);
		cv::imshow("ROI", hsvRoi);

		//erode(blobs, blobs, Mat());
		//dilate(blobs, blobs, Mat());
		//dilate(blobs, blobs, Mat());
		//dilate(blobs, blobs, Mat());

		//
		//cv::GaussianBlur(blobs, filtered, Size(7, 7), 3, 3);
		//cv::Canny(filtered, canny, 20, 35);


		/*
		Point pt;
		drawContur(output, imgG, Scalar(0, 255, 0), pt);
		drawContur(output, imgY, Scalar(0, 255, 255), pt);
		drawContur(output, imgR, Scalar(0, 0, 255), pt);
		drawContur(output, imgB, Scalar(255, 0, 0), pt);
		*/
		//output.setTo(Scalar(0, 0, 0));
		//correctedImage.copyTo(output);
		//cvCopy(&correctedImage, &output);



		cv::SimpleBlobDetector::Params params;
		params.minDistBetweenBlobs = 50.0f;
		params.filterByInertia = true;
		params.filterByConvexity = true;
		params.filterByColor = false;
		params.filterByCircularity = true;
		params.filterByArea = true ;
		params.blobColor = 0;
		params.minArea = (float)(params.minDistBetweenBlobs * 2 * CV_PI);
		params.maxArea = params.minArea * 20.f;
		params.minInertiaRatio = 0.15f;
		params.maxInertiaRatio = 1.0f;
		params.maxConvexity = 1.0f;
		params.minConvexity = 0.15f;
		params.minCircularity = 0.2f;
		params.maxCircularity = 1.0f;

		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

		// detect!
		vector<cv::KeyPoint> keypoints;
		detector->detect(hsvRoi, keypoints);
		//output = grayImage;
		grayImage.copyTo(output);

		auto count = keypoints.size();
		if (count < 2 || count > 6) {
			printf("Count error\r\n");
			//cv::waitKey(0);
		}

		float minRadius = 10000.f;
		for (auto& point : keypoints) {

			minRadius = MIN(point.size, minRadius);
		}
		float threshRadius = minRadius * 1.2f;
		vector<cv::KeyPoint> markers;
		vector<cv::KeyPoint> buttons;

		for (auto& point : keypoints) {

			if (point.size > threshRadius) {
			
				buttons.push_back(point);
			}
			else {
			
				markers.push_back(point);
			}
		}

		for (auto& point : markers)	{
			
			circle(output, point.pt, (int)(point.size / 2.f), Scalar(0, 0, 0), 3, 8, 0);
		}

		for (auto& point : buttons) {

			circle(output, point.pt, (int)(point.size / 2.f), Scalar(255, 255, 255), 3, 8, 0);
		}

		if (markers.size() != 2) {

			printf("Remote not detected\r\n");
		}
		else {

			for (auto& button : buttonPressed) {

				button = 1;
			}


			auto normDistance = cv::norm(markers[0].pt - markers[1].pt);
			//printf("Distance between markers: %f\r\n", distance);

			for (auto& point : buttons) {

				auto distance = cv::norm(point.pt - markers[0].pt) / normDistance;
				//printf("Distance between button and marker: %f\r\n", distance);

				uint8_t counter = 0;
				for (auto& dist : distances) {
					
					if(distance > dist[0] && distance < dist[1]) {
						buttonPressed[counter] = 0;
						break;
					}

					counter++;
				}

			}
		
		}

		uint8_t counter = 0;
		for (auto& button : buttonPressed) {
			counter++;
			if (button > 0) {
				printf("Button %d pressed!\r\n", counter);
				break;
			}
		}

		cv::imshow("circles", output);
		//cv::imshow("blobs", blobs);
		//cv::imshow("canny", canny);
		if (27 == cv::waitKey(100))
			break;
	}


}

int main(int argc, char** argv)
{
	//std::string video = "C:\\temp\\nrem.mjpeg";
	std::string video = "C:\\temp\\figures.mjpeg";
	//std::string video = "C:\\temp\\f2.jpg";

	cv::Mat readImage;
	cv::Mat rgbaImage;
	cv::Mat grayImage;
	cv::Mat correctedImage;

	BoardRectificationConfig brc;
	brc.analyzeRoiScale = 1.f;
	brc.sourceRoiHeight = 200.f / 720.f;	// 200 / 800
	brc.sourceRoiWidth = 350.f / 1280.f;	// 200 / 1280
	brc.outputImageHeight = 720;
	brc.outputImageWidth = 1024;
	brc.cannyTh = 330;
	brc.houghTh = 40;
	brc.houghMinLen = 30;
	brc.houghMaxGap = 5;
	BoardRectification BR(brc);

	DiceDetectionConfig ddc;
	ddc.cannyTh1 = 220;
	ddc.cannyTh2 = 220;
	ddc.fillThLow = 35;
	ddc.fillThHigh = 70;
	ddc.maxEyeDistance = 40.f;
	ddc.scaleFactX = 2.00f;
	ddc.scaleFactY = 2.00f;
	ddc.subSample = 3;
	DiceDetection dc(ddc);

	cv::Mat dice;

	Rect rectGreen;
	Rect rectRed;
	Rect rectBlue;
	Rect rectYellow;

	colorDescription dGreen;
	colorDescription dRed;
	colorDescription dBlue;
	colorDescription dYellow;

	dGreen.lowerThreshold = Scalar(60, 125, 70);
	dGreen.upperThreshold = Scalar(72, 220, 160);
	dYellow.lowerThreshold = Scalar(15, 145, 155);
	dYellow.upperThreshold = Scalar(30, 255, 255);
	dBlue.lowerThreshold = Scalar(105, 180, 130);
	dBlue.upperThreshold = Scalar(115, 255, 200);
	dRed.lowerThreshold = Scalar(179, 150, 145);
	dRed.upperThreshold = Scalar(3, 230, 200);

	RemoteDetectionConfig rdc;
	rdc.scaleFactX = 0.5f;
	rdc.scaleFactY = 0.5f;
	rdc.buttonDistances.push_back(Vec2f(1.40f, 1.80f));
	rdc.buttonDistances.push_back(Vec2f(2.50f, 2.90f));
	rdc.buttonDistances.push_back(Vec2f(3.60f, 4.00f));
	rdc.buttonDistances.push_back(Vec2f(4.70f, 5.10f));
	RemoteDetection rd(rdc);

	//Mat rectCopy;

	uint8_t repeat = 0;

#if 1
	/// adb forward tcp:8080 tcp:8080
	cv::VideoCapture cap;
	//if (cap.open("http://localhost:8080/video")) {
	if (cap.open(video)) {
		while (1) {

			if (repeat == 0 || readImage.empty()) {

				cap >> readImage;
				if (readImage.rows == 0) {

					cap.open(video);
					cap >> readImage;
				}
			}

			//cv::cvtColor(readImage, rgbaImage, CV_BGR2RGBA);
			cv::cvtColor(readImage, grayImage, CV_BGR2GRAY);
			cv::cvtColor(readImage, rgbaImage, CV_RGB2RGBA);
			//cv::cvtColor(readImage, grayImage, CV_RGB2GRAY);

			if (BR.updateBoard(grayImage)) {
				BR.rectifyImage(rgbaImage, correctedImage);
				//correctedImage.copyTo(rectCopy);

				Rect diceRect = Rect(400, 540, 200, 140);
				Mat diceImg;
				cv::cvtColor(correctedImage, diceImg, CV_BGR2GRAY);
				dice = correctedImage(diceRect);

				uint32_t dices = 0;
				uint32_t eyes = 0;
				dc.processImage(dice);
				dc.getResult(dices, eyes);
				char diceEval[64];
				sprintf(diceEval, "Dice: %d, Eyes: %d", dices, eyes);
				dc.drawBoxes(dice);

				Mat HsvImage;
				//cv::cvtColor(correctedImage, HsvImage, CV_RGB2HSV);
				cv::cvtColor(correctedImage, HsvImage, CV_BGR2HSV);

				if (locateFigure(dGreen, HsvImage, rectGreen)) {
					cv::rectangle(correctedImage, rectGreen, Scalar(0, 255, 0), 3);
				}

				if (locateFigure(dRed, HsvImage, rectRed)) {
					cv::rectangle(correctedImage, rectRed, Scalar(0, 0, 255), 3);
				}

				if (locateFigure(dBlue, HsvImage, rectBlue)) {
					cv::rectangle(correctedImage, rectBlue, Scalar(255, 0, 0), 3);
				}

				if (locateFigure(dYellow, HsvImage, rectYellow)) {
					cv::rectangle(correctedImage, rectYellow, Scalar(0, 255, 255), 3);
				}

				cv::putText(correctedImage, diceEval, Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(0, 255, 0), 2);

				colorDescription rds;
				rds.lowerThreshold = Scalar(177, 150, 105);
				rds.upperThreshold = Scalar(3, 200, 170);
				uint8_t buttonId = 0;
				char cButton[32];

				//if (REMOTE_BUTTON_PRESSED == rd.getRemoteStatus(rds, HsvImage, buttonId)) {
				if (REMOTE_BUTTON_PRESSED == rd.getRemoteStatus(rds, correctedImage, buttonId)) {
					sprintf(cButton, "Button = %d", buttonId);
				}
				else {
					sprintf(cButton, "Button = none");
				}

				cv::putText(correctedImage, cButton, Point(20, 640), FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(0, 255, 0), 2);
				//cv::imshow("Video", correctedImage);
				auto key = waitKey(1);
				if (key == 'a') {

					repeat ^= 1;
				}
			}
		}
	}
#elif 1

	int iLowH = 0, iHighH = 0, iLowS = 0, iHighS = 0, iLowV = 0, iHighV = 0;

	cv::VideoCapture cap;
	//if (cap.open("http://localhost:8080/video")) {
	if (cap.open(video)) {
		while (1) {
			cap >> cameraImage;
			if (cameraImage.rows == 0) {
				cap.open(video);
				cap >> cameraImage;
			}


			if (BR.updateBoard(cameraImage)) {

				BR.rectifyImage(cameraImage, correctedImage);
				Mat hsvImage;
				Mat output;
				output = correctedImage.clone();

				cvtColor(correctedImage, hsvImage, CV_BGR2HSV);

				//imshow("Corrected", correctedImage);

				imshow("output", output);
#if 1
				createTrackbar("LowH", "output", &iLowH, 179); //Hue (0 - 179)
				createTrackbar("HighH", "output", &iHighH, 179);

				createTrackbar("LowS", "output", &iLowS, 255); //Saturation (0 - 255)
				createTrackbar("HighS", "output", &iHighS, 255);

				createTrackbar("LowV", "output", &iLowV, 255);//Value (0 - 255)
				createTrackbar("HighV", "output", &iHighV, 255);
#endif	

				Mat imgG, imgY, imgR, imgB;

				Mat tmp;
				cvtColor(correctedImage, tmp, cv::COLOR_BGR2GRAY);
				cvtColor(tmp, output, cv::COLOR_GRAY2BGR);

				/*
				cv::Rect dstRect;
				figureDescription figure;
				figure.lowerThreshold = LowG;
				figure.upperThreshold = HighG;
				locateFigure(figure, hsvImage, dstRect);
				*/

				//while (1)
				{
					//imshow("Corrected", correctedImage);

					inRange(hsvImage, LowG, HighG, imgG);
					inRange(hsvImage, LowY, HighY, imgY);
					inRange(hsvImage, LowR, HighR, imgR);
					inRange(hsvImage, LowB, HighB, imgB);

					inHsvRange(hsvImage,
						Scalar(iLowH, iLowS, iLowV),
						Scalar(iHighH, iHighS, iHighV),
						imgG);

					morphOps(imgG);
					morphOps(imgY);
					morphOps(imgR);
					morphOps(imgB);

					correctedImage.copyTo(output, imgG);
					correctedImage.copyTo(output, imgY);
					correctedImage.copyTo(output, imgR);
					correctedImage.copyTo(output, imgB);

					Point pt;
					drawContur(output, imgG, Scalar(0, 255, 0), pt);
					drawContur(output, imgY, Scalar(0, 255, 255), pt);
					drawContur(output, imgR, Scalar(0, 0, 255), pt);
					drawContur(output, imgB, Scalar(255, 0, 0), pt);

					//output.setTo(Scalar(0, 0, 0));
					//correctedImage.copyTo(output);
					//cvCopy(&correctedImage, &output);


					imshow("output", output);
					if (27 == waitKey(100))
						break;
				}
			}
		}
	}
#endif


	exit(1);

#if 0
	// Sharpen
	//cv::GaussianBlur(cameraImage, processedImage, cvSize(9, 9), 10.0f);
	//cv::addWeighted(cameraImage, 1.5f, processedImage, -0.5f, 0, processedImage);

	// Unsharpen
	//cv::GaussianBlur(cameraImage, tempImage, cvSize(5, 5), 3.0f);
	//cv::bilateralFilter(cameraImage, tempImage, 5.0f, 50.f, 50.f);

	// Denoise (MUCH TO SLOW!)
	//cv::fastNlMeansDenoisingColored(cameraImage, tempImage, 3, 3, 7, 21);

	//adjustBrightnessContrast(cameraImage, tempImage, 1.f, 2);

	cv::cvtColor(cameraImage, grayImage, CV_RGB2GRAY);
	cv::bilateralFilter(grayImage, tempImage, 5.0f, 250.f, 250.f);
	//cvCvtColor(im_rgb, im_gray, CV_RGB2GRAY);

	float scaleFact = (float)cameraImage.size().width / 1024.0f;
	int newWidth = cameraImage.size().width / (int)scaleFact;
	int newHeight = cameraImage.size().height / (int)scaleFact;

	cv::resize(tempImage, processedImage, cvSize(newWidth, newHeight), 0.0f, 0.0f, cv::INTER_CUBIC);

	//adjustBrightnessContrast(processedImage, tempImage, 1.1f, 25);
	//cv::equalizeHist(processedImage, tempImage);
	tempImage = processedImage;
	
	cv::Mat edges, cdst;
	cv::Canny(tempImage, edges, 180.f, 360.f);
	cvtColor(edges, cdst, CV_GRAY2BGR);

	cv::Mat edges2;
	cv::dilate(edges, edges2, cv::Mat());
	//cv::morphologyEx(edges2, edges, cv::MORPH_CLOSE, cv::Mat());
	//cv::erode(edges2, edges, cv::Mat());
	//edges = edges2;

	edges = edges2(cv::Rect(0, 0, 300, 300));

	std::vector<cv::Vec4i> lines;
	HoughLinesP(edges, lines, 1, CV_PI / 180, 75, 50, 10);
	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Vec4i l = lines[i];
		cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1, CV_AA);
	}

	for (int i = 0; i < lines.size(); i++)
	{
		cv::Vec4i ol = lines[i];

		for (int j = i + 1; j < lines.size(); j++)
		{

			cv::Vec4i il = lines[j];

			cv::Point2i pt;
			//intersection(ol, il, pt);

			if (getIntersectionPoint(
				Point(ol[0], ol[1]),
				Point(ol[2], ol[3]),
				Point(il[0], il[1]),
				Point(il[2], il[3]),
				pt
				)
				) {

				cv::line(cdst, pt, pt, cv::Scalar(255, 0, 0), 3, CV_AA);
			}

			/*cv::Point2f pt = cv::computeIntersectionOfTwoLine(lines[i], lines[j]);
			if (pt.x >= 0 && pt.y >= 0 && pt.x < image.cols && pt.y < image.rows)
			{
				corners.push_back(pt);
			}
			*/
		}
	}



	cv::imshow("Original", tempImage);
	cv::imshow("Processed", edges);
	cv::imshow("Hough", cdst);

	cv::waitKey(0);
#endif
	return 0;
}