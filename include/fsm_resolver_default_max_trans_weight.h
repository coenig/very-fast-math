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

class FSMResolverDefaultMaxTransWeight : public FSMResolverDefault
{
public:
   static constexpr auto RESOLVER_NAME_MAX_TRANS_WEIGHT = "MAX_TRANS_WEIGHT";

   FSMResolverDefaultMaxTransWeight(const float threshold) // Todo: avoid magic "(" and ")".
      : FSMResolverDefault(std::string(RESOLVER_NAME_MAX_TRANS_WEIGHT) + "(" + std::to_string(threshold) + ")"), threshold_(threshold) {};

   virtual std::shared_ptr<Term> generateClosedForm() override;
   virtual std::vector<std::shared_ptr<vfm::fsm::FSMTransition>> sortTransitions(std::vector<std::shared_ptr<FSMTransition>>& transitions, const bool break_after_first_found) override;
   virtual std::string getNdetMechanismName() override;
   float getThreshold() const;

private:
   float threshold_;
};

} // fsm
} // vfm