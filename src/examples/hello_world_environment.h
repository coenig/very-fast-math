//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "simulation/master_scheduler.h"
#include <vector>
#include <string>
#include <thread>
#include <chrono>


namespace vfm {

class HelloWorldEnvironment : public MasterScheduler {

public:
   HelloWorldEnvironment();
   void handleEvent(const std::shared_ptr<EASEvent> e, const std::shared_ptr<Wink> lastActionTime) override;
   void step(const std::shared_ptr<Wink> simTime) override;
   Image getBirdseyeView() const override;
   bool isTerminationRequested(const std::shared_ptr<Wink> currentTime) override;

private:
   Image img_;
};

} // vfm
