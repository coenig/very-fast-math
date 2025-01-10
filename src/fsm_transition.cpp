//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "fsm_transition.h"
#include "term.h"

using namespace vfm;
using namespace vfm::fsm;

std::string vfm::fsm::FSMTransition::toString(const std::map<int, std::string> state_id_to_state_name) const
{
   std::string source_state = state_id_to_state_name.count(state_source_) ? state_id_to_state_name.at(state_source_) : std::to_string(state_source_);
   std::string destination_state = state_id_to_state_name.count(state_destination_) ? state_id_to_state_name.at(state_destination_) : std::to_string(state_destination_);

   return source_state + " " + FSM_PROGRAM_TRANSITION_DENOTER + " " + destination_state + " " + FSM_PROGRAM_TRANSITION_SEPARATOR + " " + condition_->serializePlainOldVFMStyle();
}

FSMTransition::FSMTransition(
   const int source_state,
   const int dest_state,
   const std::shared_ptr<Term>& cond,
   const std::shared_ptr<DataPack>& data)
   : state_source_(source_state), state_destination_(dest_state), condition_(cond), data_(data)
{}

std::ostream& operator<<(std::ostream &os, FSMTransition const &m) {
   os << m.toString();
   return os;
}
