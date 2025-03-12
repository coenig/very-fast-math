//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/mc_types.h"
#include "static_helper.h"

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

VarValsFloat vfm::MCTrace::getDeltaFromTo(const int step_a, const int step_b, const std::set<std::string> variables) const
{
   VarValsFloat res{};

   for (const auto& var : variables) {
      auto vala = getLastValueOfVariableAtStep(var, step_a);
      auto valb = getLastValueOfVariableAtStep(var, step_b);
      float delta{};

      if (StaticHelper::isParsableAsFloat(vala) && StaticHelper::isParsableAsFloat(valb)) {
         delta = std::stof(valb) - std::stof(vala);
      }
      else {
         delta = vala != valb; // For now, we return 0 if the non-float values are equal, and 1 in all other cases.
      }

      addNote("Calculating delta for variable '" + var + "' between '" + vala + "' and '" + valb + "' ==> '" + std::to_string(delta) + "'.");

      res.insert({ var, delta });
   }

   return res;
}

std::string vfm::MCTrace::getLastValueOfVariableAtStep(const std::string& var, const int step) const
{
   for (int i = std::min(step, (int) (trace_.size() - 1)); i >= 0; i--) {
      if (at(i).second.count(var)) return at(i).second.at(var);
   }

   addError("No value stored for variable '" + var + "' up to step '" + std::to_string(step) + "'.");

   return "#Variable-" + var + "-not-found";
}
