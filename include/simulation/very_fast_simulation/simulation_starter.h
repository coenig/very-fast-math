//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "simulation_time.h"
#include "par_collection.h"
#include "failable.h"
#include <vector>
#include <string>


namespace vfm {

class SimulationStarter : public Failable
{
public:
   SimulationStarter(const std::shared_ptr<ParCollection> params);
};

inline SimulationStarter::SimulationStarter(const std::shared_ptr<ParCollection> params) : Failable("SimulationStarter")
{
   if (!params) {
      addError("No parameters given.");
   }

   std::shared_ptr<std::map<std::shared_ptr<MasterScheduler>, std::shared_ptr<SimulationTime>>> timers 
      = std::make_shared<std::map<std::shared_ptr<MasterScheduler>, std::shared_ptr<SimulationTime>>>();

   std::vector<std::shared_ptr<std::vector<std::shared_ptr<Plugin>>>> runs = params->getPlugins();

   for (const auto run : runs) {
      std::shared_ptr<MasterScheduler> masterScheduler = run->at(0)->toMasterIfApplicable();
      auto simT = std::make_shared<SimulationTime>(params, run, timers);
      simT->requestNotification(3.14159);
      timers->insert({ masterScheduler, simT });
   }

   for (const auto& pair : *timers) {
      pair.second->timeStart();
   }

   addNote("Simulation started. Simulating " + std::to_string(runs.size()) + " runnable(s).");
}

}