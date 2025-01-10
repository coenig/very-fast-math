//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "fsm_resolver_default_max_trans_weight.h"
#include "static_helper.h"
#include "term.h"

std::shared_ptr<vfm::Term> vfm::fsm::FSMResolverDefaultMaxTransWeight::generateClosedForm()
{
   std::shared_ptr<vfm::Term> closed_form = nullptr; // TODO! Closed form not yet available.
   return closed_form;
}

std::vector<std::shared_ptr<vfm::fsm::FSMTransition>> vfm::fsm::FSMResolverDefaultMaxTransWeight::sortTransitions(std::vector<std::shared_ptr<FSMTransition>>& transitions, const bool break_after_first_found)
{
   // Note that we actually re-sort the transitions in the FSM.
   std::sort(transitions.begin(), transitions.end(), StaticHelper::compareTransitionsHighestWeight);
   std::vector<std::shared_ptr<FSMTransition>> transitions_out;

   for (const auto& trans : transitions) {
      if (trans->condition_->eval(data_, parser_) >= threshold_) { // TODO: Don't recalculate, store!
         transitions_out.push_back(trans);
      }
   }

   return transitions_out;
}

std::string vfm::fsm::FSMResolverDefaultMaxTransWeight::getNdetMechanismName()
{
   return "max-weight(" + std::to_string(threshold_) + ")";
}

float vfm::fsm::FSMResolverDefaultMaxTransWeight::getThreshold() const
{
   return threshold_;
}
