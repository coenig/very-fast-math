//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "fsm_resolver.h"
#include "fsm_transition.h"
#include "data_pack.h"
#include "static_helper.h"


vfm::fsm::FSMResolver::FSMResolver(const std::string &name) : Failable("FSMResolver-" + name), resolver_name_(name)
{
}

std::vector<int> vfm::fsm::FSMResolver::retrieveNextStates(const bool break_after_first_found, const bool fill_with_redundant_state_on_deadlock)
{
   std::vector<int> states;
   auto current_state_num = data_->getValFromArray(current_state_var_name_, registration_number_);

   if (outgoing_transitions_->count(current_state_num)) {
      std::vector<std::shared_ptr<FSMTransition>> transitions = outgoing_transitions_->at(current_state_num);
      transitions = sortTransitions(transitions, break_after_first_found);

      for (const auto& trans : transitions)
      {
         states.push_back(trans->state_destination_);
      }
   }

   if (fill_with_redundant_state_on_deadlock && states.empty()) {
      states.push_back(getRedundantState(data_->getValFromArray(current_state_var_name_, registration_number_)));
   }

   return states;
}

std::string vfm::fsm::FSMResolver::getName() const
{
   return resolver_name_;
}
