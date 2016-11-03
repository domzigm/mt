/*
*
* (c) domzigm 2016 - GPLv3
* https://github.com/domzigm/mt
*
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <json.hpp>
#include "BoardCalib.h"

using namespace cv;
using namespace std;
using namespace nlohmann;

string video = "C:\\temp\\figures.mjpeg";
const string calibWindow = "Calibration";

#define WARP_CNT 4u
#define FPS 30u
#define RENDERDELAY (1000u/FPS)
#define ZOOMRATIO 4u

int32_t imageWidth = 0;
int32_t imageHeight = 0;
const int32_t outputImageWidth = 1024;
const int32_t outputImageHeight = 640;

uint8_t warpLock = 0;
uint8_t warpCounter = 0;
Point2f warpPts[WARP_CNT];

Mat warp_mat;
Mat readImage;
Mat calibImage;

vector<rectangle_t> rects;
rectangle_t area;

Rect2i zoomRoi;
uint8_t zoomin = 0;

inline void validateCoord(int32_t& val, int32_t max)
{
	if (val > max) val = max;
	else if (val < 0) val = 0;
}

void mouseCb(int event, int x, int y, int flags, void* userdata)
{
	static uint8_t stateM1 = 0;

	validateCoord(x, imageWidth);
	validateCoord(y, imageHeight);

	if (0 == warpLock) {

		if (CV_EVENT_LBUTTONDOWN == event) {

			if (zoomin) {

				x = (x / ZOOMRATIO) + zoomRoi.x;
				y = (y / ZOOMRATIO) + zoomRoi.y;
			}

			Point2f* cp = &warpPts[warpCounter];
			cp->x = (float)x;
			cp->y = (float)y;
			warpCounter++;

			circle(calibImage, Point2f(cp->x, cp->y), 5, Scalar(0, 0, 255), 2);

			if (WARP_CNT == warpCounter) {

				Point2f dstPts[4];
				dstPts[0] = Point2f(0, 0);
				dstPts[1] = Point2f((float)outputImageWidth, 0);
				dstPts[2] = Point2f((float)outputImageWidth, (float)outputImageHeight);
				dstPts[3] = Point2f(0, (float)outputImageHeight);

				warp_mat = getPerspectiveTransform(warpPts, dstPts);
				warpLock = 1;
			}

			zoomin = 0;
		}
		else if (CV_EVENT_RBUTTONDOWN == event) {

			// Set / Move the zooming ROI
			zoomin = 1;
			auto dstSize = calibImage.size();

			zoomRoi.x = x - dstSize.width   / ZOOMRATIO;
			zoomRoi.y = y - dstSize.height  / ZOOMRATIO;
			zoomRoi.width  = dstSize.width  / ZOOMRATIO;
			zoomRoi.height = dstSize.height / ZOOMRATIO;
			
			if (zoomRoi.x < 0) zoomRoi.x = 0;
			if (zoomRoi.y < 0) zoomRoi.y = 0;
		}

	}
	else {

		Rect2f* r = &area.rect;
		if (CV_EVENT_LBUTTONDOWN == event && 0 == stateM1) {

			r->x = (float)x;
			r->y = (float)y;

			area.rect.width = 0;
			area.rect.height = 0;
			area.color = Scalar(rand() % 255, rand() % 255, rand() % 255);

			stateM1 = 1;
		}
		else if (CV_EVENT_LBUTTONUP == event && 1 == stateM1) {
			
			if (x < r->x) {

				float tmp = r->x;
				r->x = (float)x;
				x = (int)tmp;
			}
			
			if (y < r->y) {

				float tmp = r->y;
				r->y = (float)y;
				y = (int)tmp;
			}

			r->width = x - r->x;
			r->height = y - r->y;

			if (r->width > 10 && r->height > 10) {

				rects.push_back(area);
			}

			*r = Rect2f(0, 0, 0, 0);

			stateM1 = 0;
		}
		else if (CV_EVENT_RBUTTONDOWN == event) {

			for(auto it = rects.begin(); it != rects.end();	++it) {
				
				Rect2f click = { (float)x, (float)y, 1.f, 1.f };
				Rect2f tmp = it->rect;

				if ( (tmp & click) == click) {
					rects.erase(it);
					break;
				}
			}
		}
		else if (CV_EVENT_MBUTTONDOWN == event) {

			for (auto it = rects.begin(); it != rects.end(); ++it) {

				Rect2f click = { (float)x, (float)y, 1.f, 1.f };
				Rect2f tmp = it->rect;

				if ((tmp & click) == click) {
					cout << "Please enter name for area at " << it->rect.x << "x" << it->rect.y << " : ";
					cin.sync();
					cin >> it->name;
					break;
				}
			}
		}
		else if (CV_EVENT_MOUSEMOVE == event) {

			area.rect.width = x - area.rect.x;
			area.rect.height = y - area.rect.y;
		}
	}
}

void dumpJson(string fileName)
{
	json jsonBoard;
	uint32_t ctr = 0;

	for (auto it = rects.begin(); it != rects.end(); ++it) {

		json jsObj;
		jsObj["id"] = ctr++;
		jsObj["name"] = it->name;
		jsObj["eventmask"] = "0";
		jsObj["fixedevent"] = "0";
		jsObj["coords"]["x"] = it->rect.x / (float)outputImageWidth;
		jsObj["coords"]["y"] = it->rect.y / (float)outputImageHeight;
		jsObj["coords"]["width"] = it->rect.width / (float)outputImageWidth;
		jsObj["coords"]["height"] = it->rect.height / (float)outputImageHeight;

		jsonBoard["BoardRegions"].push_back(jsObj);
	}
	try {

		ofstream outputFile(fileName);
		outputFile << jsonBoard.dump(4);
		outputFile.close();
		cout << "Map data exported to file " << fileName << endl;
	}
	catch (...)	{

		cout << "Error while exporting map data!" << endl;
	}
}

int main(int argc, char** argv)
{
	cv::VideoCapture cap;

	if (argc == 2) {

		video = argv[1];
	}

	if (cap.open(video)) {

		cap >> readImage;
	}
	else {

		return 0;
	}

	cout << "Instructions" << endl;
	cout << "====================" << endl;
	cout << "Right mouse button to zoom in" << endl;
	cout << "Left mouse button to mark the corners, starting upper left in clock rotation" << endl;
	cout << endl;
	cout << "After calibration" << endl;
	cout << "====================" << endl;
	cout << "Left mouse button to drag-create an area" << endl;
	cout << "Middle mouse button to name an area (using the console)" << endl;
	cout << "Right mouse button to remove an area" << endl;

	imageWidth = readImage.cols;
	imageHeight = readImage.rows;

	readImage.copyTo(calibImage);
	namedWindow(calibWindow);
	setMouseCallback(calibWindow, mouseCb);
	Mat zoom;
	int key = 0;

	while (1) {

		if (warpLock) {
			
			warpPerspective(readImage, calibImage, warp_mat, Size(outputImageWidth, outputImageHeight));

			for (auto& rt : rects) {

				// Draw bounding box and fill with transparent color
				Mat roi = calibImage(rt.rect);
				Mat overlay(roi.rows, roi.cols, CV_8UC3);
				overlay.setTo(rt.color);
				addWeighted(roi, 0.5f, overlay, 0.5f, 0, roi);
				rectangle(roi, Point(0, 0), Point(roi.cols - 1, roi.rows - 1), rt.color, 2);
				putText(roi, rt.name, Point(5, 15), FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(0, 255, 0), 1);
				
			}

			if (area.rect.x && area.rect.y) {

				rectangle(calibImage, area.rect, area.color, 2);
			}
		}
		else {

			if (zoomin) {

				resize(calibImage(zoomRoi), zoom, calibImage.size());
				imshow(calibWindow, zoom);
				key = waitKey(RENDERDELAY);
				continue;
			}

		}

		imshow(calibWindow, calibImage);
		key = waitKey(RENDERDELAY);

		if (key == 'd') {

			dumpJson("Board.json");
		}
	}

	return 0;
}
