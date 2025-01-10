//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "examples/hello_world_environment.h" // This is just a temporary dummy version, no dependence to examples intended, of course.
#include "plugin.h"
#include <vector>
#include <string>


namespace vfm {

const std::string ACTION_ON_UNCAUGHT_EXCEPTION_LOG_AND_RECOVER = "logAndRecover";
const std::string ACTION_ON_UNCAUGHT_EXCEPTION_ASK_WHAT_TO_DO = "askWhatToDo";
const std::string ACTION_ON_UNCAUGHT_EXCEPTION_PRINT_AND_HALT = "printErrorMessageAndHalt";
const std::string ACTION_ON_UNCAUGHT_EXCEPTION_PRINT_AND_TERMINATE = "printErrorMessageAndTerminate";
const std::string ACTION_ON_UNCAUGHT_EXCEPTION_REMOVE_SCHEDULER_AND_RECOVER = "removeMaliciousPluginAndRecover";
const std::string ACTION_ON_UNCAUGHT_EXCEPTION_DO_NOTHING = "doNothing";

/**
* A dummy parameter collection to be filled with actual dynamic parameters later.
*/
class ParCollection {

public:

   std::vector<std::shared_ptr<std::vector<std::shared_ptr<Plugin>>>> getPlugins();

   inline std::string getActionOnUncaughtException() {
      return ACTION_ON_UNCAUGHT_EXCEPTION_LOG_AND_RECOVER;
   }

   inline bool isForceExitAfterSimulationTerminates() {
      return false;
   }

private:

};

inline std::vector<std::shared_ptr<std::vector<std::shared_ptr<Plugin>>>> ParCollection::getPlugins()
{
   auto vec1 = std::make_shared<std::vector<std::shared_ptr<Plugin>>>();
   auto vec2 = std::make_shared<std::vector<std::shared_ptr<Plugin>>>();
   vec2->push_back(std::make_shared<HelloWorldEnvironment>());
   return { vec1, /*vec2*/ };
}

} // vfm
