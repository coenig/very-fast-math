//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "fsm_singleton_generator.h"


using namespace vfm::fsm;

std::vector<std::shared_ptr<FSMs>> FSMsSingletonGenerator::instances_;

std::shared_ptr<FSMs> FSMsSingletonGenerator::getInstance(const int id)
{
   while (instances_.size() <= id) {
      auto new_instance = std::make_shared<FSMs>();
      instances_.push_back(new_instance);
   }

   return instances_[id];
}
