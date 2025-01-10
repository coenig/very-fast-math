//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "fsm_resolver.h"

namespace vfm {

namespace fsm {

class FSMResolverDefault : public FSMResolver
{
public:
   static constexpr auto RESOLVER_NAME_DEFAULT = "DEFAULT";

   FSMResolverDefault() : FSMResolver(RESOLVER_NAME_DEFAULT) {};

   /// The default closed form is resolving non-determinism as follows:
   /// - No transition active: Go to initial state.
   /// - More than one transition active: Go to reachable state with highest ID.
   ///
   /// The closed form is based on this sub-formula F(t) for a single transition t with condition c 
   /// from state s to state s':
   ///      F(t) = (c & s) * s'
   /// All these transition formulae are combined using the max operator, including a single "INVALID_STATE_NUM"
   /// that ensures that the initial state is entered if no explicit transition has a "true" condition:
   ///      F(t1) max F(t2) max F(t3) max ... max INVALID_STATE_NUM
   virtual std::shared_ptr<Term> generateClosedForm() override;

   virtual int getRedundantState(const int from) override;
   virtual std::vector<std::shared_ptr<vfm::fsm::FSMTransition>> sortTransitions(std::vector<std::shared_ptr<FSMTransition>>& transitions, const bool break_after_first_found) override;
   virtual std::string getDeadlockMechanismName() override;
   virtual std::string getNdetMechanismName() override;

protected:
   FSMResolverDefault(const std::string& name) : FSMResolver(name) {};
};

} // fsm
} // vfm