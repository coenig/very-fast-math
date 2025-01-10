//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include <vector>
#include <string>
#include <memory>


namespace vfm {

class SimulationTime;

/**
* Represents a moment in the course of simulation time.
* Every notification sent from the simulation engine (i.e., from a
* <code>SimulationTime</code> object) is associated to a wink which either
* represents the exact moment in simulation time when this notification
* was fired or, in the case of events, the moment of the last regular,
* i.e., non-event, notification.
*/
class Wink {

public:

   /**
   * Constructor.
   * 
   * @param time    The time elapsed since the start of the simulation.
   * @param isTick  Iff the stored time is concurrent with a tick.
   */
   Wink(const double time, const bool isTick, std::shared_ptr<SimulationTime> simTimeObject);

   /**
   * @return Returns if the wink time is a tick.
   */
   bool isTick() const;

   /**
   * @return Returns the currentTime.
   */
   double getExactTime() const;

   /**
   * Use carefully, this may corrupt the flow of time in {@link SimulationTime}.
   * 
   * @param currentTime The currentTime to set.
   */
   void setCurrentTime(const double currentTime);

   std::string toString() const;

   /**
   * @return  The last exceeded tick, this is, (long) currentTime.
   */
   long getLastTick() const;

   Wink copy() const;

   /**
   * @return Returns the simTime objects that belongs to this wink.
   */
   std::shared_ptr<SimulationTime> getSimTime() const; /// Note that this will return a nullptr before the SimulationTime started running.

   /**
   * Sets the simTime objects that belongs to this wink.
   */
   void setSimTime(const std::shared_ptr<SimulationTime> simTime);

   /**
   * @param tick The tick to set.
   */
   void setTick(const bool tick);

   /**
   * Sets the current time to the next tick 
   * (this is, Math.floor(this.currentTime + 1)).
   * The variable isTick is set to true.
   */
   void nextTick();

private:

   /**
   * Iff the stored time is concurrent with a tick.
   */
   bool isTick_;

   /**
   * The time stored in this wink.
   */
   double currentTime_;

   /**
   * The simulation time object that belongs to this wink.
   */
   std::shared_ptr<SimulationTime> simTime_;
};

inline Wink::Wink(const double time, const bool isTick, std::shared_ptr<SimulationTime> simTimeObject) :
   isTick_(isTick),
   currentTime_(time),
   simTime_(simTimeObject)
{}

inline bool Wink::isTick() const 
{
   return isTick_;
}

inline double Wink::getExactTime() const 
{
   return currentTime_;
}

inline void Wink::setCurrentTime(const double currentTime) 
{
   currentTime_ = currentTime;
}

inline std::string Wink::toString() const 
{
   if (isTick_) {
      return std::to_string(getLastTick());
   } else {
      return std::to_string(getExactTime());
   }
}

inline long Wink::getLastTick() const 
{
   return (long) currentTime_;
}

inline Wink Wink::copy() const
{
   return Wink(currentTime_, isTick_, simTime_);
}

inline std::shared_ptr<SimulationTime> Wink::getSimTime() const
{
   return simTime_;
}

inline void Wink::setSimTime(const std::shared_ptr<SimulationTime> simTime)
{
   simTime_ = simTime;
}

inline void Wink::setTick(const bool tick) {
   isTick_ = tick;
}

inline void Wink::nextTick() {
   currentTime_ = ((long) currentTime_) + 1;
   isTick_ = true;
}

} // vfm
