//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "event.h"
#include "master_scheduler.h"
#include "par_collection.h"
#include "plugin.h"
#include "wink.h"
#include "failable.h"
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <ctime>
#include <chrono>
#include <thread>
#include <functional>
#include <limits>
#include <algorithm>


namespace vfm {

using Date = std::chrono::system_clock::time_point;
using EventFilter = std::function<bool(const std::shared_ptr<EASEvent> e)>;

const auto EVENT_FILTER_TRUE = [](const std::shared_ptr<EASEvent> e) { return true; };
const auto EVENT_FILTER_FALSE = [](const std::shared_ptr<EASEvent> e) { return false; };

/**
* Implementation of the simulation engine.</BR>
* </BR>
* Note that the terms 'plugin' and 'scheduler' are used synonymously. The 
* master scheduler is a special type of plugin.</BR>
* </BR>
* The simulation engine sends notifications to the registered plugins which
* can decide dynamically what to pass on to the EASRunnables (usually environments and agents. 
* This architectural choice is specific to EAS establishing an intermediate layer 
* between the simulation engine and the simulation model.</BR>
* </BR>
* Application developers do not need to use this class frequently in an 
* explicit manner. An exception is, for example, the event broadcasting
* method. Other than that, instances of this class work more or less
* in the background.
*/
class SimulationTime : public std::enable_shared_from_this<SimulationTime>, public Failable {

public:

   /**
   * Constructor.
   * 
   * @param runnable  The runnable simulated. Note that due to the EAS policy of separating
   *                  the simulation engine from the simulation model, the {@link SimulationTime}
   *                  will never directly call any methods of the runnable, but only give it
   *                  as an argument to accordingly registered plugins. 
   * @param plugins   Plugin objects to be triggered during simulation.
   * @param params    Runtime parameters (allows to specify parameters other than
   *                  {@link GlobalVariables#getParameters()} - which is the 99%-default, though).
   * @param allTimers Allows a reverse match of available {@link EASRunnable}s to according {@link SimulationTime}s.
   */
   SimulationTime(
      const std::shared_ptr<ParCollection> params,
      const std::shared_ptr<std::vector<std::shared_ptr<Plugin>>> plugs, 
      const std::shared_ptr<std::map<std::shared_ptr<MasterScheduler>, std::shared_ptr<SimulationTime>>> allTim);

   void neglectTimeStep(const std::shared_ptr<Plugin> p, const double timeStep);
   std::vector<double> getAllTimeStepsToBeNotified(const std::shared_ptr<Plugin> p);

   /**
   * @return Returns the startTime.
   */
   Date getStartTime();

   /**
   * Resets the simulation time to the beginning.
   */
   void reset();

   /**
   * Runs the complete simulation cycle.
   */
   void run();

   bool allTerminated();

   /**
   * @return Returns the timeFinished.
   */
   bool isTimeTerminated();

   /**
   * Checks if all required plugins (by any plugin) are existing and throws 
   * Exception if there are missing ones.
   */
   void checkRequiredPlugins();

   /**
   * Sets the current time to the next time step that is relevant for 
   * notifications if there exists such a time step within simulation
   * time. If the flag stopSim is true, nothing is done and 
   * <code>false</code> is returned.
   * 
   * @return  Returns <code>true</code> as long as some plugin is still to
   *          be notified, <code>false</code> otherwise.
   */
   bool setNextNotificationTime();

   /**
   * The plugins that have to be notified in the current time step are
   * notified. If there are notifications left to do within simulation time,
   * the current simulation time is set to the next time step that is
   * associated to a notification; otherwise, <code>false</code> is returned
   * meaning that the simulation should be stopped. Note that
   * 
   * @return  Returns <code>true</code> as long as some plugin is still to
   *          be notified, <code>false</code> otherwise.
   */
   void step();

   /**
   * Runs the course of time which, in turn, keeps running in a separate thread.
   */
   void timeStart();

   /**
   * Requests a termination of simulation time. This will occur in the next
   * 
   */
   void timeTerminate();

   /**
   * Retrieves the first plugin of a given name, if present.
   * 
   * @param id  The name of a plugin as given by the {@link Plugin#id()} method.
   * 
   * @return  The first plugin in the plugins list of <code>this</code> run 
   *          with the given name. <code>null</code>, if there is no plugin 
   *          with the given name.<BR/>
   *          <BR/>
   *          There is no way to retrieve the second or third plugin of the given
   *          name if there is more than one. For this, use method.
   *          {@link #getAllPluginsByName(String)}.
   */
   std::shared_ptr<Plugin> getPluginByName(const std::string& id);

   /**
   * Retrieves all plugins of a given name.
   * 
   * @param id  The name of a plugin as given by the {@link Plugin#id()} method.
   * 
   * @return  A list of all plugins with the given name, in the order they
   *          have been registered to <code>this</code> {@link SimulationTime}.
   *          Usually, there is only one plugin or none at all of a specific
   *          type (=name); in this case, the list is empty or has one
   *          single element.
   */
   std::vector<std::shared_ptr<Plugin>> getAllPluginsByName(const std::string& id);

   /**
   * @return Returns the currentTime object holding the time step the 
   *         simulation is currently in.
   */
   std::shared_ptr<Wink> getCurrentWink();

   /**
   * Gives a notification to the given Plugin at every tick from now on
   * until the neglectTicks-method is called.
   * 
   * @param requester  The plugin that should be notified.
   */
   void requestTicks(const std::shared_ptr<Plugin> requester);

   /**
   * Stops notifying the given plugin at every tick until the requestTicks-
   * method is called.
   * 
   * @param requester  The plugin to ignore at ticks.
   */
   void neglectTicks(const std::shared_ptr<Plugin> requester);

   /**
   * Requests a notification for the requesting plugin at a specific
   * time step in the future. CAUTION! Note that time steps requested using
   * this method are NOT CONSIDERED TICKS - even if they happen to point at
   * a time simultaneous to a tick.
   * 
   * @param requester  The plugin to be notified.
   * @param time       The time of notification.
   */
   void requestNotification(const std::shared_ptr<Plugin> requester, const double time);

   /**
   * Gives a notification to this SimulationTime's master scheduler 
   * at every tick from now on until the neglectTicks-method is called.
   * 
   * @param requester  The plugin that should be notified.
   */
   void requestTicks();

   /**
   * Stops notifying this SimulationTime's master scheduler 
   * at every tick until the requestTicks-method is called.
   * 
   * @param requester  The plugin to ignore at ticks.
   */
   void neglectTicks();

   /**
   * Requests a notification for this SimulationTime's master scheduler at 
   * a specific time step. CAUTION! Note that time steps requested using
   * this method are NOT CONSIDERED TICKS - even if they happen to point at
   * a time simultaneous to a tick.
   * 
   * @param requester  The plugin to be notified.
   * @param time       The time of notification.
   */
   void requestNotification(const double time);

   /**
   * Requests a notification for any time step a notification occurs in
   * the simulation.
   * This influences only the invokation of the method runDuringSim.
   * The given plugin is invoked AFTER the invokation of other
   * plugins at the same time step.
   * Note: plugins that have been invoked at a time step are not invoked
   * in the same time step a second time.
   * 
   * @param requester  The requesting plugin. Note that there is no separate
   *                   method for the master scheduler as master schedulers
   *                   typically will not need this functionality. If a 
   *                   master scheduler M should need this, the method can be 
   *                   invoked as requestAllNotifications(M).
   */
   void requestAllNotifications(const std::shared_ptr<Plugin> requester);

   /**
   * Neglects notifications for all time steps a notification occurs in
   * the simulation (cf. inverse method {@link #requestAllNotifications()}).
   * 
   * @param requester  The neglecting plugin. Note that there is no separate
   *                   method for the master scheduler as master schedulers
   *                   typically will not need this functionality. If a 
   *                   master scheduler {@code M} should need this, the method can be 
   *                   invoked as {@code requestAllNotifications(M)}.
   */
   void neglectAllNotifications(const std::shared_ptr<Plugin> requester);

   void requestAllNotifications();

   void neglectAllNotifications();

   /**
   * Ignores the plugin for all future notifications including events. The 
   * plugin cannot recover by itself from this state. Another scheduler 
   * can request future notifications for this scheduler, though.
   * 
   * @param requester  The scheduler (plugin) to be ignored.
   */
   void neverNotifyAgain(const std::shared_ptr<Plugin> requester);

   /**
   * @return  This SimulationTime's master scheduler.
   */
   std::shared_ptr<MasterScheduler> getMasterScheduler() const;

   /**
   * Broadcasts the given event to all registered plugins.
   * 
   * @param e  The event causing the broadcast.
   */
   void broadcastEvent(const std::shared_ptr<EASEvent> e);

   /**
   * Requests events for the specified plugin filtered by the specified
   * event filter.
   * 
   * @param plugin  The plugin to be notified.
   * @param filter  The event filter.
   */
   void requestEvents(const std::shared_ptr<Plugin> plugin, const EventFilter& filter);

   /**
   * Requests events for the master scheduler filtered by the specified
   * event filter.
   * 
   * @param filter  The event filter.
   */
   void requestEvents(const EventFilter& filter);

   /**
   * Requests ALL events for the specified plugin (filtered by the constant
   * event filter <code>EVENT_FILTER_TRUE</code>).
   * 
   * @param plugin  The plugin to be notified.
   */
   void requestAllEvents(const std::shared_ptr<Plugin> plugin);

   /**
   * Requests ALL events for the master scheduler (filtered by the constant
   * event filter <code>EVENT_FILTER_TRUE</code>).
   */
   void requestAllEvents();

   /**
   * Add a plugin to the list of registered plugins. The plugin is put at
   * the end of the list and therefore notified last. Ticks are automatically
   * requested for the new plugin, and as a first action, the plugin's
   * <code>runBeforeSimulation</code> method is called. Everything else has 
   * to be done from outside.  (The complementary method to remove a plugin 
   * is called <code>neverNotifyAgain</code>.)
   * 
   * @param plugin  The plugin to register.
   * @return  If the plugin has been registered (if not, probably a required
   *          plugin is missing).
   */
   bool registerPlugin(const std::shared_ptr<Plugin> plugin);

   std::shared_ptr<std::vector<std::shared_ptr<Plugin>>> getPlugins();

   std::vector<std::shared_ptr<Plugin>> getAllSleepingPlugins();

   /**
   * @return Returns all timers, i.e., a mapping of all runnables to be
   *         simulated concurrently to their respective {@link SimulationTime}
   *         objects.
   */
   std::shared_ptr<std::map<std::shared_ptr<MasterScheduler>, std::shared_ptr<SimulationTime>>> getAllTimers();

   /**
   * Synchronizes this {@link SimulationTime} object with possible 
   * simultaneously running simulations. More precisely, the thread this
   * simulation is running in will be delayed (in sequential chunks of 5ms)
   * until all other simulations (if any) catch up to the {@link #currentTime}
   * of this simulation or later.
   */
   void synchronize();

private:
   std::shared_ptr<ParCollection> pars;
   std::vector<std::shared_ptr<Plugin>> pluginsToRegisterAfterExecutionOfCurrentStep;
   std::set<std::shared_ptr<Plugin>> neverNotify;
   bool ignoreAllExceptions = false;
   bool timeTerminated = false;

   /**
   * Set of plugins that are notified at all notifications that 
   * occur for any other plugin - NOT at events.
   * This influences only the invokation of the method runDuringSim.
   * The plugins in this set are invoked AFTER the invokation of other
   * plugins at the same time step.
   * Note: plugins that have already been invoked at a time step are not invoked
   * in the same time step for a second time.
   */
   std::set<std::shared_ptr<Plugin>> toBeNotifiedAtAllNotifications;

   /**
   * A list of all plugins requesting events. Every plugin is
   * assigned to an event filter deciding if the plugin should be notified
   * for a specific event broadcast.
   */
   std::map<std::shared_ptr<Plugin>, EventFilter> eventRequestingPlugins;

   std::shared_ptr<std::map<std::shared_ptr<MasterScheduler>, std::shared_ptr<SimulationTime>>> allTimers;
   std::shared_ptr<Wink> currentTime;

   /**
   * Iff the simulation should stop at the next possible step.
   */
   bool stopSim;

   /**
   * List of plugin objects to manipulate simulation. First plugin is master
   * scheduler.
   */
   std::shared_ptr<std::vector<std::shared_ptr<Plugin>>> plugins;

   /**
   * The environment being simulated.
   */
   //std::shared_ptr<EASRunnable> runnable;

   /**
   * The thread for the simulation time to run in.
   */
   std::shared_ptr<std::thread> simThread;

   /**
   * Iff ticks are requested by any plugin.
   */
   bool isRequestedTicks;

   /**
   * For every plugin the information if ticks are requested or not.
   */
   std::map<std::shared_ptr<Plugin>, bool> requestedTicks;
   Date startTime;

   /**
   * For requested notification time the plugins that requested it.
   * Keys are sorted.
   */
   std::map<double, std::vector<std::shared_ptr<Plugin>>> requestedNotifications;

   /**
   * @return  Iff this simulation is "ahead" of any other simultaneously running simulation.
   *          I.e., if this simulation ran faster than another simulation and is now at a
   *          time step ahead of it, this method will return {@code true}, {@code false} otherwise.
   */
   bool isAhead();

   std::string forRunnableText() const;

   bool isPluginSleeping(const std::shared_ptr<Plugin> p);
   void runBeforeOrAfterSimulation(const std::shared_ptr<Plugin> p, const bool before = true);
   void registerPluginSynchronized(const std::shared_ptr<Plugin> p);

   /**
   * Handles cases where an uncaught exception occurred in a plugin.
   * 
   * @param e  The uncaught exception.
   * @param p  The plugin / scheduler that caused the exception.
   */
   void handleExceptionInMainLoop(const std::exception& e, const std::shared_ptr<Plugin> p);
};

inline SimulationTime::SimulationTime(
   const std::shared_ptr<ParCollection> params,
   const std::shared_ptr<std::vector<std::shared_ptr<Plugin>>> plugs, 
   const std::shared_ptr<std::map<std::shared_ptr<MasterScheduler>, std::shared_ptr<SimulationTime>>> allTim)
   : Failable("SimulationTime"),
   pars(params),
   //pluginsToRegisterAfterExecutionOfCurrentStep,
   //std::map<Plugin> neverNotify,
   //ignoreAllExceptions = false,
   //timeTerminated = false,
   //toBeNotifiedAtAllNotifications,
   //eventRequestingPlugins,
   allTimers(allTim),
   //currentTime,
   //stopSim,
   plugins(plugs),
   //runnable(runnble),
   //simThread,
   //isRequestedTicks,
   //requestedTicks,
   startTime(std::chrono::system_clock::now())
   //requestedNotifications 
{
   assert(plugs && !plugs->empty());

   for (const auto p : *plugins) {
      requestTicks(p);
   }

   reset();
   checkRequiredPlugins();
}

inline void SimulationTime::neglectTimeStep(const std::shared_ptr<Plugin> p, const double timeStep)
{
   auto plugins_at_time_step = requestedNotifications.at(timeStep);

   for (int i = 0; i < plugins_at_time_step.size(); i++) {
      const auto plugin = plugins_at_time_step[i];

      if (plugin->id() == p->id()) {
         plugins_at_time_step.erase(plugins_at_time_step.begin() + i);
      }
   }
}

inline std::vector<double> SimulationTime::getAllTimeStepsToBeNotified(const std::shared_ptr<Plugin> p)
{
   std::vector<double> list;

   for (const auto& pair : requestedNotifications) {
      auto plugins = pair.second;

      for (const auto& plugin : plugins)
         if (p->id() == plugin->id()) {
            list.push_back(pair.first);
         }
   }

   return list;
}

inline Date SimulationTime::getStartTime()
{
   return startTime;
}

inline std::string SimulationTime::forRunnableText() const
{
   return "for master '" + getMasterScheduler()->id() + "'";
}

inline void SimulationTime::reset()
{
   stopSim = false;
   currentTime = std::make_shared<Wink>(-1, true, nullptr); // A negative currentTime indicates a simulation before the start of time.
   addNote("Reset to the initial state " + forRunnableText() + ".");
}

inline void SimulationTime::run()
{
   currentTime->setSimTime(shared_from_this());

   addNote("Course of time started " + forRunnableText() + ".");

   // Starting the plugin method before Simulation.
   for (const auto p : *plugins) {
      try {
         if (!neverNotify.count(p)) {
            runBeforeOrAfterSimulation(p);
         }
      } catch (const std::exception& e) {
         handleExceptionInMainLoop(e, p);
      }
   }

   bool fortsetzen = setNextNotificationTime();

   // Main loop generating time and event notifications for plugins.
   while (fortsetzen) {
      step();
      fortsetzen = setNextNotificationTime();
   }

   // Starting the plugin after Simulation.
   for (const auto p : *plugins) {
      try {
         if (!neverNotify.count(p)) {
            runBeforeOrAfterSimulation(p, false);
         }
      } catch (const std::exception& e) {
         handleExceptionInMainLoop(e, p);
      }
   }

   addNote("Course of time ended " + forRunnableText() + ".");

   timeTerminated = true;

   if (pars->isForceExitAfterSimulationTerminates() && allTerminated()) {
      std::exit(0);
   }
}

inline bool SimulationTime::allTerminated()
{
   for (const auto& t : *allTimers) {
      if (!t.second->isTimeTerminated()) {
         return false;
      }
   }

   return true;
}

inline bool SimulationTime::isTimeTerminated()
{
   return timeTerminated;
}

inline void SimulationTime::checkRequiredPlugins()
{
   bool gefunden;

   if (!plugins) {
      return; // No plugins means none required.
   }

   for (const auto p : *plugins) {
      for (const auto& prStr : p->getRequiredPlugins()) {
         gefunden = false;
         for (const auto p2 : *plugins) {
            if (prStr == p2->id()) {
               gefunden = true;
               break;
            }
         }

         if (!gefunden) {
            addError("Plugin '" + p->id() + "' requires plugin '" + prStr + "' which could not be found.");
         }
      }
   }
}

inline bool SimulationTime::setNextNotificationTime()
{
   if (!stopSim && 
      (isRequestedTicks || !requestedNotifications.empty())) {
      long nextTick = currentTime->getLastTick() + 1;
      double nextNot = std::numeric_limits<double>::infinity();

      if (!requestedNotifications.empty()) {
         nextNot = requestedNotifications.begin()->first;
      }

      // Do nothing if time is terminated.
      if (getMasterScheduler()->isTerminationRequested(currentTime)) {
         return false;
      }

      if (isRequestedTicks && nextTick < nextNot) {
         // isTick is set automatically.
         currentTime->nextTick();
      } else {
         currentTime->setCurrentTime(nextNot);
         currentTime->setTick(false);
      }

      return true;
   }

   return false;
}

inline void SimulationTime::step()
{
   std::set<std::shared_ptr<Plugin>> invoked;

   if (currentTime->isTick()) {
      for (const auto p : *plugins) {
         if (requestedTicks.at(p)) {
            try {
               p->step(currentTime);
            } catch (const std::exception& e) {
               handleExceptionInMainLoop(e, p);
            }

            invoked.insert(p);
         }
      }
   } else { // Time step is no tick, i.e., a requested notification.
      for (const auto p : requestedNotifications.at(currentTime->getExactTime())) {
         try {
            p->step(currentTime);
         } catch (const std::exception& e) {
            handleExceptionInMainLoop(e, p);
         }

         invoked.insert(p);
      }

      requestedNotifications.erase(currentTime->getExactTime());
   }

   /* 
   * Invoke all plugins that want to be notified every time anything 
   * happens.
   */
   if (invoked.size() > 0) {
      for (const auto p : toBeNotifiedAtAllNotifications) {
         if (!invoked.count(p)) {
            try {
               p->step(currentTime);
            } catch (const std::exception& e) {
               handleExceptionInMainLoop(e, p);
            }
         }
      }
   }

   /*
   * Register new waiting plugins.
   */
   for (const auto p : pluginsToRegisterAfterExecutionOfCurrentStep) {
      registerPluginSynchronized(p);
   }

   pluginsToRegisterAfterExecutionOfCurrentStep.clear();
}

inline void SimulationTime::timeStart()
{
   auto fct = [this]() { this->run(); };
   simThread = std::make_shared<std::thread>(fct);
}

inline void SimulationTime::timeTerminate()
{
   stopSim = true;
}

inline std::shared_ptr<Plugin> SimulationTime::getPluginByName(const std::string & id)
{
   auto plugins = getAllPluginsByName(id);

   if (plugins.empty()) {
      return nullptr;
   }

   return plugins[0];
}

inline std::vector<std::shared_ptr<Plugin>> SimulationTime::getAllPluginsByName(const std::string & id)
{
   std::vector<std::shared_ptr<Plugin>> plugs;

   if (plugins) {
      for (const auto& p : *plugins) {
         if (p->id() == id) {
            plugs.push_back(p);
         }
      }
   }

   return plugs;
}

inline std::shared_ptr<Wink> SimulationTime::getCurrentWink()
{
   return currentTime;
}

inline void SimulationTime::requestTicks(const std::shared_ptr<Plugin> requester)
{
   requestedTicks.insert({ requester, true });
   isRequestedTicks = true;
}

inline void SimulationTime::neglectTicks(const std::shared_ptr<Plugin> requester)
{
   requestedTicks.insert({ requester, false });

   for (const auto& pair : requestedTicks) {
      if (pair.second) {
         return;
      }
   }

   isRequestedTicks = false;
}

inline void SimulationTime::requestNotification(const std::shared_ptr<Plugin> requester, const double time)
{
   if (time <= currentTime->getExactTime()) {
      return;
   }

   //        if (time > this->pars->getExperimentLength()) {
   //            StaticMethods.log(
   //                    StaticMethods.LOG_WARNING,
   //                    "Requested notification time "
   //                    + time + " (for " + requester.id() + ") "
   //                    + "is greater than the total simulation time (" 
   //                    + this->pars->getExperimentLength() + ").",
   //                    this->pars);
   //        }

   if (time < 0) {
      addError("Requested notification time "
         + std::to_string(time) + " (for " + requester->id() + ") is before simulation start.");
   }

   if (!requestedNotifications.count(time)) {
      requestedNotifications.insert({ time, {} });
   }

   auto& currList = requestedNotifications.at(time);

   if (std::find(currList.begin(), currList.end(), requester) == currList.end()) {
      currList.push_back(requester);
   }
}

inline void SimulationTime::requestTicks()
{
   requestTicks(getMasterScheduler());
}

inline void SimulationTime::neglectTicks()
{
   neglectTicks(getMasterScheduler());
}

inline void SimulationTime::requestNotification(const double time)
{
   requestNotification(getMasterScheduler(), time);
}

inline void SimulationTime::requestAllNotifications(const std::shared_ptr<Plugin> requester)
{
   toBeNotifiedAtAllNotifications.insert(requester);
}

inline void SimulationTime::neglectAllNotifications(const std::shared_ptr<Plugin> requester)
{
   toBeNotifiedAtAllNotifications.erase(requester);
}

inline void SimulationTime::requestAllNotifications()
{
   requestAllNotifications(getMasterScheduler());
}

inline void SimulationTime::neglectAllNotifications()
{
   neglectAllNotifications(getMasterScheduler());
}

inline void SimulationTime::neverNotifyAgain(const std::shared_ptr<Plugin> requester)
{
   neverNotify.insert(requester);
   neglectAllNotifications(requester);
   neglectTicks(requester);
   eventRequestingPlugins.erase(requester);

   for (const auto& pair : requestedNotifications) {
      std::vector<std::shared_ptr<Plugin>>& list = requestedNotifications.at(pair.first);
      list.erase(std::remove(list.begin(), list.end(), requester), list.end());
   }
}

inline std::shared_ptr<MasterScheduler> SimulationTime::getMasterScheduler() const
{
   return std::dynamic_pointer_cast<MasterScheduler>(plugins->at(0));
}

inline void SimulationTime::broadcastEvent(const std::shared_ptr<EASEvent> e)
{
   for (const auto& pair : eventRequestingPlugins) {
      if (pair.second(e)) {
         pair.first->handleEvent(e, currentTime);
      }
   }
}

inline void SimulationTime::requestEvents(const std::shared_ptr<Plugin> plugin, const EventFilter & filter)
{
   eventRequestingPlugins.insert({ plugin, filter });
}

inline void SimulationTime::requestEvents(const EventFilter & filter)
{
   requestEvents(getMasterScheduler(), filter);
}

inline void SimulationTime::requestAllEvents(const std::shared_ptr<Plugin> plugin)
{
   requestEvents(plugin, EVENT_FILTER_TRUE);
}

inline void SimulationTime::requestAllEvents()
{
   requestEvents(EVENT_FILTER_TRUE);
}

inline bool SimulationTime::registerPlugin(const std::shared_ptr<Plugin> plugin)
{
   for (const auto& pID : plugin->getRequiredPlugins()) {
      bool gefunden = false;

      for (const auto p : *plugins) {
         if (pID == p->id()) {
            gefunden = true;
         }
      }

      if (!gefunden) {
         addWarning("Plugin '" + plugin->id() + "' has NOT been registered as the required plugin '"
            + pID + "' is not registered.");
         return false;
      }
   }

   if (pluginsToRegisterAfterExecutionOfCurrentStep.empty()) {
      pluginsToRegisterAfterExecutionOfCurrentStep.push_back(plugin);
      return true;
   } else {
      addWarning("Plugin '" + plugin->id() + "' has NOT been registered because another plugin ('"
         + pluginsToRegisterAfterExecutionOfCurrentStep.at(0)->id()
         + "') is waiting to be registered.");
      return false;
   }
}

inline std::shared_ptr<std::vector<std::shared_ptr<Plugin>>> SimulationTime::getPlugins()
{
   return plugins;
}

inline std::vector<std::shared_ptr<Plugin>> SimulationTime::getAllSleepingPlugins()
{
   std::vector<std::shared_ptr<Plugin>> sleeping;

   for (const auto p : *plugins) {
      if (isPluginSleeping(p)) {
         sleeping.push_back(p);
      }
   }

   return sleeping;
}

inline std::shared_ptr<std::map<std::shared_ptr<MasterScheduler>, std::shared_ptr<SimulationTime>>> SimulationTime::getAllTimers()
{
   return allTimers;
}

inline void SimulationTime::synchronize()
{
   while (isAhead()) {
      try {
         std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } catch (const std::exception& e) {
         addError("Synchronization of master '" + getMasterScheduler()->id() + "' failed at time " + currentTime->toString() + ".\n" + e.what());
      }
   }
}

inline bool SimulationTime::isAhead()
{
   for (const auto& pair : *allTimers) {
      if (pair.second->currentTime->getExactTime() < currentTime->getExactTime()) {
         return true;
      }
   }

   return false;
}

inline bool SimulationTime::isPluginSleeping(const std::shared_ptr<Plugin> p)
{
   return neverNotify.count(p) && !eventRequestingPlugins.count(p);
}

inline void SimulationTime::runBeforeOrAfterSimulation(const std::shared_ptr<Plugin> p, const bool before)
{
   std::string bef = before ? "Initializ" : "Finaliz";
   addNote(bef + "ing plugin '" + p->id() + "' " + forRunnableText() + "...");

   if (before) {
      p->runBeforeSimulation(currentTime);
   }
   else {
      p->runAfterSimulation(currentTime);
   }

   addNote(bef + "ed plugin  '" + p->id() + "' " + forRunnableText() + ".");
}

inline void SimulationTime::registerPluginSynchronized(const std::shared_ptr<Plugin> p)
{
   requestTicks(p);
   runBeforeOrAfterSimulation(p);
   plugins->push_back(p);
}

inline void SimulationTime::handleExceptionInMainLoop(const std::exception& e, const std::shared_ptr<Plugin> p)
{
   std::string time = std::to_string(currentTime->getExactTime());

   if (ignoreAllExceptions) {
      return;
   }

   if (pars->getActionOnUncaughtException() == ACTION_ON_UNCAUGHT_EXCEPTION_ASK_WHAT_TO_DO) {
      addNote("User interaction dialog needs to be implemented, yet. Will log error message and recover instead.");
   } else if (pars->getActionOnUncaughtException() == ACTION_ON_UNCAUGHT_EXCEPTION_DO_NOTHING) {
      return;
   }

   std::string master = "";

   if (p->id() == getMasterScheduler()->id()) {
      master = " (MASTER SCHEDULER)";
   }

   // Log message.
   if (pars->getActionOnUncaughtException() != ACTION_ON_UNCAUGHT_EXCEPTION_DO_NOTHING) {
      addError("Main simulation loop caught at time step "
         + time
         + " an unhandled exception thrown by scheduler '"
         + p->id()
         + "'"
         + master
         + ": \n---\n"
         + e.what());
   }

   if (pars->getActionOnUncaughtException() == ACTION_ON_UNCAUGHT_EXCEPTION_REMOVE_SCHEDULER_AND_RECOVER) {
      if (master.length() == 0) { // Ignore plugin in future (except master scheduler).
         neverNotifyAgain(p);
         addNote("Scheduler '" + p->id() + "' has been removed from simulation.");
      }
      else {
         addNote("Scheduler '" + p->id() + "' has NOT been removed from simulation since it is the master scheduler.");
      }
   } else if (pars->getActionOnUncaughtException() == ACTION_ON_UNCAUGHT_EXCEPTION_PRINT_AND_TERMINATE) { // Terminate.
      addNote("Simulation is being TERMINATED at end of current cycle.");
      timeTerminate();
   } else if (pars->getActionOnUncaughtException() == ACTION_ON_UNCAUGHT_EXCEPTION_PRINT_AND_HALT) { // Halt immediately.
      addNote("Simulation is being HALTED immediately. Goodbye!");
      std::exit(0);
   }

   addNote("Course of time has recovered from an uncaught exception.");
}

} // vfm
