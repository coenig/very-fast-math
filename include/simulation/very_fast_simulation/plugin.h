//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "wink.h"
#include "event.h"
#include "failable.h"
#include <vector>
#include <string>


namespace vfm {


class MasterScheduler;

/**
* Father class for the implementation of plugins (schedulers) in EAS. 
* The main methods are called (1) before the beginning of, (2) during or 
* (3) after the termination of the simulation, resp., or as a reaction to an
* EASevent which can occur at "any time".
*/
class Plugin : public Failable {

public:

   /**
   * Constructor. The parameter is a textual identification string of this plugin. 
   * The identification has to be unique, i.e., two different plugins have
   * to have two different ids.
   */
   Plugin(const std::string& plugin_id);

   /**
   * Dependencies between plugins. If any plugin from this list is inactive,
   * this plugin cannot be used in a simulation run.
   * 
   * @return The list of required plugins. Can be <code>null</code>
   *         if no dependencies are present.
   */
   virtual std::vector<std::string> getRequiredPlugins()
   {
      return {};
   }

   /**
   * This method is called before the first simulation cycle. 
   * All initiation processes are finished at this time.
   * 
   * @param env  The runnable that this plugin is called on.
   */
   virtual void runBeforeSimulation(const std::shared_ptr<Wink> currentSimTime);

   /**
   * This method is called after the last simulation cycle. 
   * All simulation processes are still running at this time. Termination
   * is initiated directly after the call of all runAfterSimulation
   * methods of the active plugins.
   * 
   * @param env     The runnable that this plugin is called on.
   */
   virtual void runAfterSimulation(const std::shared_ptr<Wink> currentSimTime);

   /**
   * This method is called during some simulation cycle 
   * <code>currentSimTime</code>. 
   * 
   * @param env             The runnable that this plugin is called on.
   * @param currentSimTime  The current point in time.
   */
   virtual void step(const std::shared_ptr<Wink> currentSimTime);

   /**
   * Handles events for this plugin.
   * 
   * @param event        Some EASEvent.
   * @param env          The environment (more precisely main runnable) to 
   *                     which the plugin belongs.
   * @param lastSimTime  The last time step before the event occured.
   */
   virtual void handleEvent(const std::shared_ptr<EASEvent> event, const std::shared_ptr<Wink> lastSimTime);

   virtual std::shared_ptr<MasterScheduler> toMasterIfApplicable();

   std::string id() const;

private:
   const std::string id_;
};

inline Plugin::Plugin(const std::string& plugin_id) : Failable("PLUGIN " + plugin_id), id_(plugin_id)
{
}

inline void Plugin::runBeforeSimulation(const std::shared_ptr<Wink> currentSimTime)
{
   addNote("runBeforeSimulation executed. Override the method for custom behavior.");
}

inline void Plugin::runAfterSimulation(const std::shared_ptr<Wink> currentSimTime)
{
   addNote("runAfterSimulation executed. Override the method for custom behavior.");
}

inline void Plugin::step(const std::shared_ptr<Wink> currentSimTime)
{
   addNote("runDuringSimulation executed at time step " + currentSimTime->toString() + ". Override the method for custom behavior.");
}

inline void Plugin::handleEvent(const std::shared_ptr<EASEvent> event, const std::shared_ptr<Wink> lastSimTime)
{
   addNote("Plugin has been notified about the event '"
      + event->getEventDescription() + "', but no action has been taken.\n"
      + "To catch the event, override the Plugin's handleEvent(.) method.");
}

inline std::shared_ptr<MasterScheduler> Plugin::toMasterIfApplicable()
{
   return nullptr;
}

inline std::string Plugin::id() const
{
   return id_;
}

} // vfm
