#include <stdint.h>
#include "json.hpp"
#include "ConfigParser.h"

using namespace nlohmann;

namespace mt
{

uint8_t boardRectificationCfgParser(const char* jsonStr, mt::BoardRectificationConfig& config)
{
	uint8_t retVal = 0;
	json jsObj;
	jsObj.parse(jsonStr);
	mt::BoardRectificationConfig cfg;

	try {
		auto js = jsObj["BoardRectification"];
		cfg.analyzeRoiScale = js["AnalyzeRoiScale"];
		cfg.cannyTh = js["CannyThreshold"];
		cfg.houghMaxGap = js["HoughMaxGap"];
		cfg.houghMinLen = js["HoughMinLen"];
		cfg.houghTh = js["HoughThreshold"];
		cfg.outputImageHeight = js["OutputHeight"];
		cfg.outputImageWidth = js["OutputWidth"];
		cfg.sourceRoiHeight = js["RoiHeight"];
		cfg.sourceRoiWidth = js["RoiWidth"];
		config = cfg;
	}
	catch (...) {
		retVal = 1;
	}

	return retVal;
}

uint8_t diceDetectionCfgParser(const char* jsonStr, mt::DiceDetectionConfig& config)
{
	uint8_t retVal = 0;
	json jsObj;
	jsObj.parse(jsonStr);
	mt::DiceDetectionConfig cfg;

	try {
		auto js = jsObj["DiceDetection"];
		cfg.cannyTh1 = js["CannyThreshold1"];
		cfg.cannyTh2 = js["CannyThreshold2"];
		cfg.fillThHigh = js["FillThresholdHigh"];
		cfg.fillThLow = js["FillThresholdLow"];
		cfg.maxEyeDistance = js["MaxEyeDistance"];
		cfg.scaleFactX = js["ScalingX"];
		cfg.scaleFactY = js["ScalingY"];
		cfg.subSample = js["Subsample"];
		config = cfg;
	}
	catch (...) {
		retVal = 1;
	}

	return retVal;
}

uint8_t remoteDetectionCfgParser(const char* jsonStr, mt::RemoteDetectionConfig& config)
{
	uint8_t retVal = 0;
	json jsObj;
	jsObj.parse(jsonStr);
	mt::RemoteDetectionConfig cfg;

	try {
		auto js = jsObj["RemoteDetection"];
		cfg.scaleFactX = js["ScalingX"];
		cfg.scaleFactY = js["ScalingY"];
		auto distances = js["Distances"];
		for (auto distance : distances) {
			cv::Vec2f vDist(js["MinDistance"], js["MaxDistance"]);
			cfg.buttonDistances.push_back(vDist);
		}

		config = cfg;
	}
	catch (...) {
		retVal = 1;
	}

	return retVal;
}

}
