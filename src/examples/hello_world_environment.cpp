//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "hello_world_environment.h"
#include "hello_world_environment.h"
#include "hello_world_environment.h"
#include "hello_world_environment.h"
#include "simulation/simulation_time.h"


using namespace vfm;

vfm::HelloWorldEnvironment::HelloWorldEnvironment() : MasterScheduler("HelloWorldEnvironment")
{
}

void HelloWorldEnvironment::handleEvent(const std::shared_ptr<EASEvent> e, const std::shared_ptr<Wink> lastActionTime)
{
   addNote("Event has occurred: " + e->getEventDescription());
}

void HelloWorldEnvironment::step(const std::shared_ptr<Wink> simTime)
{
   addNote("Performed step (" + id() + "): " + std::to_string(simTime->getExactTime()));
}

Image HelloWorldEnvironment::getBirdseyeView() const
{
   return img_;
}

bool vfm::HelloWorldEnvironment::isTerminationRequested(const std::shared_ptr<Wink> currentTime)
{
   return false;
}
