//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/mc_types.h"

using namespace vfm;


vfm::MCTrace::MCTrace() : Failable("MCTrace") {}

TraceStep vfm::MCTrace::at(const size_t index) const
{
   return trace_.at(index);
}

size_t vfm::MCTrace::size() const
{
   return trace_.size();
}

bool vfm::MCTrace::empty() const
{
   return trace_.empty();
}

const std::vector<TraceStep>& vfm::MCTrace::getConstTrace() const
{
   return trace_;
}

std::vector<TraceStep>& vfm::MCTrace::getTrace()
{
   return trace_;
}

void vfm::MCTrace::addTraceStep(const TraceStep& step)
{
   trace_.push_back(step);
}

VarValsFloat vfm::MCTrace::getDeltaFromTo(const int step_a, const int step_b) const
{
   VarValsFloat res{};
   VarVals a{ at(step_a).second };
   VarVals b{ at(step_b).second };

   for (const auto& varvala : a) {
      if (b.count(varvala.first)) {
         res.insert({ varvala.first, 0.0f });
      }
   }

   return res;
}
