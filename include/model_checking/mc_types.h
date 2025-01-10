//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

namespace vfm {

// {{{"1.1", {{"actuatorStateLat", "enum_actuatorState_failure"}, {"actuatorStateLong", "enum_actuatorState_failure"}, {...}}}}, {...}, {...}, ...}
using VarVals = std::map<std::string, std::string>;
using TraceStep = std::pair<std::string, VarVals>;
using MCTrace = std::vector<TraceStep>;
} // vfm