//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "math_struct.h"
#include <map>

namespace vfm { namespace fsm { class FSMTransition; } }

std::ostream& operator<<(std::ostream &os, vfm::fsm::FSMTransition const &m);

namespace vfm {

class DataPack;

namespace fsm {

class FSMTransition
{
public:
   int state_source_;
   int state_destination_;
   std::shared_ptr<Term> condition_;
   std::shared_ptr<DataPack> data_ = nullptr;

   friend std::ostream& ::operator<<(std::ostream &os, const FSMTransition& m);

   std::string toString(const std::map<int, std::string> state_id_to_state_name = std::map<int, std::string>{}) const;

   FSMTransition(
      const int source_state, 
      const int dest_state, 
      const std::shared_ptr<Term>& cond,
      const std::shared_ptr<DataPack>& data);

private:
};

using ::operator<<;

} // fsm
} // vfm