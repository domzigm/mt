/*
 *
 * (c) domzigm 2016 - GPLv3
 * https://github.com/domzigm/mt
 * 
 */
 
#pragma once

#include <stdint.h>

namespace mt
{

struct BoardRectificationConfig
{
	/**
	Note: There are four ROIs, one in each corner. The input size is based the percentage specified by sourceRoi[Width/Height]
	**/

	//! Width of the rectified image in pixels
	int outputImageWidth;
	
	//! Height of the rectified image in pixels
	int outputImageHeight;

	//! Fixed size of the roi used for analysis
	int analyzeRoiSize;

	//! The source roi width in percent
	float sourceRoiWidth;

	//! The source roi height in percent
	float sourceRoiHeight;
};

}
