//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "fsm_resolver_default.h"
#include "data_pack.h"
#include "static_helper.h"
#include "parser.h"
#include "term_max.h"
#include "term_val.h"
#include "term_var.h"
#include "term_mult.h"
#include "term_logic_eq.h"
#include "term_logic_or.h"
#include "term_logic_and.h"
#include "term.h"

std::string vfm::fsm::FSMResolverDefault::getDeadlockMechanismName()
{
   return "back-to-initial";
}

std::string vfm::fsm::FSMResolverDefault::getNdetMechanismName()
{
   return "max-id";
}

int vfm::fsm::FSMResolverDefault::getRedundantState(const int from)
{
   return initial_state_num_; // Go back to initial state if no transition is active.
}

std::vector<std::shared_ptr<vfm::fsm::FSMTransition>> vfm::fsm::FSMResolverDefault::sortTransitions(std::vector<std::shared_ptr<FSMTransition>>& transitions, const bool break_after_first_found)
{
   // Note that we actually re-sort the transitions in the FSM. 
   // TODO: Do this only once in the beginning, as the ordering is always the same.
   std::sort(transitions.begin(), transitions.end(), StaticHelper::compareTransitionsHighestDestinationID);
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

std::shared_ptr<vfm::Term> vfm::fsm::FSMResolverDefault::generateClosedForm()
{
   std::shared_ptr<Term> closed_form = _val(initial_state_num_);

   for (const auto& trans : *transitions_plain_set_)
   {
      if (trans->state_destination_ != initial_state_num_) { // To initial state we jump anyway if no other transition is active.
         std::shared_ptr<Term> cond_part1 = _eq(_val(trans->state_source_), _arr(current_state_var_name_, _val(registration_number_)));
         std::shared_ptr<Term> cond_part2 = trans->condition_->toTermIfApplicable();
         std::shared_ptr<Term> cond = _and(cond_part1, cond_part2);
         std::shared_ptr<Term> cond_mult = _mult(cond, _val(trans->state_destination_));
         closed_form = _max(closed_form, cond_mult);
      }
   }

   closed_form->setChildrensFathers(); // Probably unnecessary.

   return closed_form;
}
