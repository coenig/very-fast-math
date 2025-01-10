//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "parsable.h"
#include "fsm_resolver.h"
#include "fsm_resolver_default.h"
#include "fsm_resolver_default_max_trans_weight.h"
#include "fsm_resolver_remain_on_no_transition.h"
#include "fsm_resolver_remain_on_no_transition_and_obey_insertion_order.h"


namespace vfm {
namespace fsm {

class FSMResolverFactory : public Parsable
{
public:
   static constexpr auto RESOLVER_DENOTER = "RESOLVER";

   static std::shared_ptr<FSMResolver> createNewResolverFromExisting(const std::shared_ptr<FSMResolver> other);

   FSMResolverFactory();
   virtual bool parseProgram(const std::string& program) override;
   bool isResolverCommand(const std::string& command) const;
   std::shared_ptr<FSMResolver> getGeneratedResolver() const;

private:
   std::shared_ptr<FSMResolver> generated_resolver_ = nullptr;
};

} // fsm
} // vfm
