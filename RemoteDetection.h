/*
 *
 * (c) domzigm 2016 - GPLv3
 * https://github.com/domzigm/mt
 * 
 */

#pragma once

#include "RemoteDetectionConfig.h"
#include "HelperRoutines.h"

namespace mt
{
	enum RemoteStatus
	{
		REMOTE_NOT_DETECTED,
		REMOTE_NOT_PRESSED,
		REMOTE_BUTTON_PRESSED
	};

class RemoteDetection
{
public:

	RemoteDetection(RemoteDetectionConfig& config);
	RemoteStatus getRemoteStatus(hsvPlanes& desc, cv::Mat& srcimage, uint8_t& buttonId);

private:

	RemoteDetectionConfig&				m_config;
	cv::SimpleBlobDetector::Params		m_sbdParams;
	uint8_t								m_calibMarkerCount;
	float								m_radiusTolerance;
};

}
