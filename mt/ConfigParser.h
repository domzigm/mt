#pragma once

#include "BoardRectificationConfig.h"
#include "DiceDetectionConfig.h"
#include "RemoteDetectionConfig.h"

namespace mt
{

uint8_t boardRectificationCfgParser(const char* jsonStr, mt::BoardRectificationConfig& config);
uint8_t diceDetectionCfgParser(const char* jsonStr, mt::DiceDetectionConfig& config);
uint8_t remoteDetectionCfgParser(const char* jsonStr, mt::RemoteDetectionConfig& config);

}
