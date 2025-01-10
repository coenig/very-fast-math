//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "fsm.h"


namespace vfm {
namespace fsm {

class FSMsSingletonGenerator
{
public:
   static std::shared_ptr<FSMs> getInstance(const int id);

private:
   static std::vector<std::shared_ptr<FSMs>> instances_;
};

} // fsm
} // vfm