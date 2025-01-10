//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "fsm_resolver_default.h"


namespace vfm {
namespace fsm {

/// \brief Resolver that handles non-determinism by remaining in current state
///  if no transition is active and otherwise doing the same as FSMResolverDefault.
class FSMResolverRemainOnNoTransition : public FSMResolverDefault
{
public:
   static constexpr auto RESOLVER_NAME_REMAIN_ON_NO_TRANSITION = "REMAIN_ON_NO_TRANSITION";

   FSMResolverRemainOnNoTransition() : FSMResolverDefault(RESOLVER_NAME_REMAIN_ON_NO_TRANSITION) {};

   virtual std::shared_ptr<Term> generateClosedForm() override; /// TODO: Cannot handle more than one active transition, yet. (Remember that it has to work with nuSMV, too.)
   virtual int getRedundantState(const int from) override;
   virtual std::string getDeadlockMechanismName() override;

protected:
   FSMResolverRemainOnNoTransition(const std::string& name) : FSMResolverDefault(name) {};
};

} // fsm
} // vfm