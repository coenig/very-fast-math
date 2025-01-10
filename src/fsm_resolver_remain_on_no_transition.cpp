//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "fsm_resolver_remain_on_no_transition.h"
#include "term.h"
#include "parser.h"
#include "static_helper.h"
#include "term_plus.h"
#include "term_val.h"
#include "term_var.h"
#include "term_mult.h"
#include "term_logic_eq.h"
#include "term_logic_or.h"
#include "term_logic_and.h"

std::string vfm::fsm::FSMResolverRemainOnNoTransition::getDeadlockMechanismName()
{
   return "remain-in-current";
}

int vfm::fsm::FSMResolverRemainOnNoTransition::getRedundantState(const int from)
{
   return from;  // Stay in current state if no transition is active.
}

std::shared_ptr<vfm::Term> vfm::fsm::FSMResolverRemainOnNoTransition::generateClosedForm()
{
   std::shared_ptr<Term> closed_form = _val(0);

   for (const auto& trans : *transitions_plain_set_)
   {
      std::shared_ptr<Term> cond_part1 = _eq(_val(trans->state_source_), _arr(current_state_var_name_, _val(registration_number_)));
      std::shared_ptr<Term> cond_part2 = trans->condition_->toTermIfApplicable();
      std::shared_ptr<Term> cond = _and(cond_part1, cond_part2);
      std::shared_ptr<Term> cond_mult = _mult(cond, _val(trans->state_destination_));
      closed_form = _plus(closed_form, cond_mult);
   }

   // Handle the case of no active transition.
   auto temp_formula = _mult(_arr(current_state_var_name_, _val(registration_number_)), _not(closed_form));
   closed_form = _plus(closed_form, temp_formula);

   closed_form->setChildrensFathers(); // Probably unnecessary.

   return closed_form;
}
