//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "math_struct.h"
#include "failable.h"
#include <iostream>
#include <memory>
#include <vector>
#include <map>

namespace vfm {
class DataPack;
class Term;

namespace fsm {
class FSMTransition;

/// \brief Used by the (deterministic!) FSM to resolve ill-defined (in a strictly
/// mathematical sense) situations of non-determinism or deadlock. These two cases
/// are combined into one class since the resolver has to offer an associated 
/// closed-form representation that captures both situations simultaneously.
///
/// Note that FSMs can, in principle, be non-deterministic in the real world.
/// In this they differ to almost any other type of automaton. Non-determinism is
/// for now not implemented for the vfm::fsm framework, but might be considered 
/// in future.
class FSMResolver : public Failable
{
public:
   FSMResolver(const std::string& name);

   void setFSMData(
      const int initial_state_num,
      const std::string& current_state_var_name,
      const int registration_number,
      const std::shared_ptr<std::vector<std::shared_ptr<FSMTransition>>> transitions_plain_set,
      const std::shared_ptr<std::map<int, std::vector<std::shared_ptr<FSMTransition>>>> outgoing_transitions,
      const std::shared_ptr<std::map<int, std::vector<std::shared_ptr<FSMTransition>>>> incoming_transitions,
      const std::shared_ptr<DataPack> data,
      const std::shared_ptr<FormulaParser> parser,
      const FSMJitLevel jit_level) 
   {
       initial_state_num_ = initial_state_num;
       current_state_var_name_ = current_state_var_name;
       registration_number_ = registration_number;
       transitions_plain_set_ = transitions_plain_set;
       outgoing_transitions_ = outgoing_transitions;
       incoming_transitions_ = incoming_transitions;
       data_ = data;
       parser_ = parser;
       jit_level_ = jit_level;
   };

   /// \brief Generates a closed-form arithmetic expression which implements the transition function of
   /// the FSM. The expression can be compiled at runtime ("just-in-time") into native assembly code
   /// making evaluation extremely fast, and it is then subsequently used for calculating the next state 
   /// inside the step() method.
   ///
   /// To avoid any unnecessary operations such as nullptr checks, the method has to be called
   /// manually from outside, once the FSM is complete, i.e., when all transitions have been
   /// added. If it is not called, there is currently no fallback to the regular transition
   /// calculation mode, so the program will crash.
   ///
   /// Returns a pointer to the generated closed form.
   virtual std::shared_ptr<Term> generateClosedForm() = 0;

   /// \brief Retrieves the states that have active transitions
   /// pointing to them from the current state. This should
   /// usually be only one state, getting several states means
   /// that the transition definition is not deterministic.
   /// Setting break_after_first_found to true will resolve
   /// this in a deterministic way. Otherwise it will return some
   /// ordering of the active transitions with the first being considered
   /// the deterministic one. The ordering is given by sortTransitions(.).
   /// This method uses either no_jit or jit_cond to calculate the
   /// active transitions, it can be used in all modes, though.
   std::vector<int> retrieveNextStates(const bool break_after_first_found, const bool fill_with_redundant_state_on_deadlock = true);

   /// \brief Sorts a list of transitions descending with their priority of being taken.
   /// In the deterministic case, the 0-th element of the returned list reflects the transition to take.
   /// Note that the ingoing transitions may be sorted, but must not have elements removed or added.
   /// The returned list is therefore a copy.
   virtual std::vector<std::shared_ptr<vfm::fsm::FSMTransition>> sortTransitions(std::vector<std::shared_ptr<FSMTransition>>& transitions, const bool break_after_first_found) = 0;

   /// \brief Returns the state that is redundant to define a transition to, meaning
   /// that it would have been taken in a deadlock case anyway, even without an explicit transition.
   virtual int getRedundantState(const int from) = 0;

   /// \brief An externally printable description of this resolver's deadlock mechanism.
   virtual std::string getDeadlockMechanismName() = 0;

   /// \brief An externally printable description of this resolver's nondeterminism mechanism.
   virtual std::string getNdetMechanismName() = 0;

   std::string getName() const;

protected:
   int initial_state_num_ = -1;
   std::string current_state_var_name_ = "undefined";
   int registration_number_ = 0;
   std::shared_ptr<std::vector<std::shared_ptr<FSMTransition>>> transitions_plain_set_ = nullptr;
   std::shared_ptr<std::map<int, std::vector<std::shared_ptr<FSMTransition>>>> outgoing_transitions_ = nullptr;
   std::shared_ptr<std::map<int, std::vector<std::shared_ptr<FSMTransition>>>> incoming_transitions_ = nullptr;
   std::shared_ptr<DataPack> data_ = nullptr;
   std::shared_ptr<FormulaParser> parser_ = nullptr;
   FSMJitLevel jit_level_ = FSMJitLevel::no_jit;

private:
   std::string resolver_name_;
};

} // fsm
} // vfm