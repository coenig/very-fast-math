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

namespace mc {

   static const std::pair<std::string, std::string> TESTCASE_MODE_PREVIEW{ "preview", "preview" };
   static const std::pair<std::string, std::string> TESTCASE_MODE_PREVIEW_2{ "preview2", "preview 2" };
   static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_FULL{ "cex-full", "full (1/7)" };
   static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_BIRDSEYE{ "cex-birdseye", "birdseye (2/7)" };
   static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_COCKPIT_ONLY{ "cex-cockpit-only", "cockpit (3/7)" };
   static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_SMOOTH_FULL{ "cex-smooth-full", "full-smooth (4/7)" };
   static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_SMOOTH_BIRDSEYE{ "cex-smooth-birdseye", "birdseye-smooth (5/7)" };
   static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_SMOOTH_WITH_ARROWS_FULL{ "cex-smooth-with-arrows-full", "smooth-with-arrows (6/7)" };
   static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_SMOOTH_WITH_ARROWS_BIRDSEYE{ "cex-smooth-with-arrows-birdseye", "smooth-with-arrows-birdseye (7/7)" };

   static const std::map<std::string, std::string> ALL_TESTCASE_MODES{
      TESTCASE_MODE_PREVIEW,
      TESTCASE_MODE_PREVIEW_2,
      TESTCASE_MODE_CEX_FULL,
      TESTCASE_MODE_CEX_BIRDSEYE,
      TESTCASE_MODE_CEX_COCKPIT_ONLY,
      TESTCASE_MODE_CEX_SMOOTH_FULL,
      TESTCASE_MODE_CEX_SMOOTH_BIRDSEYE,
      TESTCASE_MODE_CEX_SMOOTH_WITH_ARROWS_FULL,
      TESTCASE_MODE_CEX_SMOOTH_WITH_ARROWS_BIRDSEYE,
   };

   static const std::vector<std::string> ALL_TEST_CASE_MODES_PLAIN_NAMES()
   {
      std::vector<std::string> keys{};

      for (const auto& pair : ALL_TESTCASE_MODES) {
         keys.push_back(pair.first);
      }

      return keys;
   }

} // mc

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
   std::vector<VarValsFloat> getAllDeltas(const std::set<std::string> variables) const;
   std::string getLastValueOfVariableAtStep(const std::string& var, const int step) const; // Retrieves a value at some step, and looks in earlier steps if the current one doesn't contain it.

private:
   std::vector<TraceStep> trace_{};
};

} // vfm
