//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "failable.h"
#include <vector>

namespace vfm {

// {{{"1.1", {{"actuatorStateLat", "enum_actuatorState_failure"}, {"actuatorStateLong", "enum_actuatorState_failure"}, {...}}}}, {...}, {...}, ...}
using VarVals = std::map<std::string, std::string>;
using VarValsFloat = std::map<std::string, float>;
using TraceStep = std::pair<std::string, VarVals>;

class MCTrace : public Failable {
public:
   MCTrace();

   TraceStep at(const size_t index) const;
   size_t size() const;
   bool empty() const;
   const std::vector<TraceStep>& getConstTrace() const;
   std::vector<TraceStep>& getTrace();
   void addTraceStep(const TraceStep& step);
   VarValsFloat getDeltaFromTo(const int step_a, const int step_b, const std::set<std::string> variables) const;
   std::string getLastValueOfVariableAtStep(const std::string& var, const int step) const; // Retrieves a value at some step, and looks in earlier steps if the current one doesn't contain it.

private:
   std::vector<TraceStep> trace_{};
};

} // vfm