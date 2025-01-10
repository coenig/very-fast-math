//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "geometry/images.h"
#include "plugin.h"
#include <vector>
#include <string>


namespace vfm {

/**
* Implementation of a master scheduler which extends the Plugin 
* class to enable the generation of a list of environments given to the SimulationTime 
* object at initialization time and simulated simultaneously in separate threads. 
* Furthermore, a master scheduler provides a method for deciding if the simulation should be
* terminated. The <code>AbstractDefaultMaster</code> class provides standard
* functionality and can be used as a foundation for most simulations.
*/
class MasterScheduler : public Plugin, public std::enable_shared_from_this<MasterScheduler> {

public:
   MasterScheduler(const std::string& master_name);

   /**
   * The termination rule for this scheduler requesting the simulation
   * time to stop.
   * 
   * @param runnable     The runnable this scheduler belongs to.
   * @param currentTime  The current simulation time.
   * @param params       The parameter collection.
   * @return  Iff the simulation should stop now.
   */
   virtual bool isTerminationRequested(const std::shared_ptr<Wink> currentTime) = 0;

   virtual Image getOutsideView() const = 0;

   virtual std::shared_ptr<MasterScheduler> toMasterIfApplicable() override;
};

inline MasterScheduler::MasterScheduler(const std::string& master_name) : Plugin(master_name)
{
}

inline std::shared_ptr<MasterScheduler> MasterScheduler::toMasterIfApplicable()
{
   return std::dynamic_pointer_cast<MasterScheduler>(shared_from_this());
}

} // vfm
