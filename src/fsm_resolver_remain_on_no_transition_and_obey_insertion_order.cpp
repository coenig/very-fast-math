//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "fsm_resolver_remain_on_no_transition_and_obey_insertion_order.h"
#include "term.h"
#include "static_helper.h"
#include "term_plus.h"
#include "term_val.h"
#include "term_var.h"
#include "term_mult.h"
#include "term_logic_eq.h"
#include "term_logic_or.h"
#include "term_logic_and.h"

std::shared_ptr<vfm::Term> vfm::fsm::FSMResolverRemainOnNoTransitionAndObeyInsertionOrder::generateClosedForm()
{
   return nullptr;
}

std::vector<std::shared_ptr<vfm::fsm::FSMTransition>> vfm::fsm::FSMResolverRemainOnNoTransitionAndObeyInsertionOrder::sortTransitions(std::vector<std::shared_ptr<FSMTransition>>& transitions, const bool break_after_first_found)
{
   std::vector<std::shared_ptr<FSMTransition>> transitions_out;

   for (const auto& trans : transitions)
   {
      bool result = trans->condition_->eval(data_, parser_);

      if (result) {
         transitions_out.push_back(trans);

         if (break_after_first_found) {
            break; // Welldefined as we sorted the transitions BEFORE.
         }
      }
   }

   return transitions_out;
}

std::string vfm::fsm::FSMResolverRemainOnNoTransitionAndObeyInsertionOrder::getNdetMechanismName()
{
   return "obey-insertion-order";
}
