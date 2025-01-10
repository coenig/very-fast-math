//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "fsm_resolver_remain_on_no_transition.h"


namespace vfm {
namespace fsm {

/// \brief Handles deadlocks by remaining in current state and nondeterminism by obeying
///  the insertion order of transitions.
class FSMResolverRemainOnNoTransitionAndObeyInsertionOrder : public FSMResolverRemainOnNoTransition
{
public:
   static constexpr auto RESOLVER_NAME_REMAIN_ON_NO_TRANSITION_AND_OBEY_INSERTION_ORDER = "REMAIN_ON_NO_TRANSITION_AND_OBEY_INSERTION_ORDER";

   FSMResolverRemainOnNoTransitionAndObeyInsertionOrder() : FSMResolverRemainOnNoTransition(RESOLVER_NAME_REMAIN_ON_NO_TRANSITION_AND_OBEY_INSERTION_ORDER) {}

   virtual std::shared_ptr<Term> generateClosedForm() override;
   virtual std::vector<std::shared_ptr<vfm::fsm::FSMTransition>> sortTransitions(std::vector<std::shared_ptr<FSMTransition>>& transitions, const bool break_after_first_found) override;
   virtual std::string getNdetMechanismName() override;
};

} // fsm
} // vfm