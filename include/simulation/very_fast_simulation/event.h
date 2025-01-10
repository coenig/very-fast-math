//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include <vector>
#include <string>


namespace vfm {

/**
* Implements an event in the EAS simulation. The event is so far defined
* by a string description only. Do not confuse events with notifications
* of progression in time. Both time notifications and events are triggered
* by a SimulationTime object, however, events are not bound to a specific
* time step, but may occur always, particularly between time steps (ticks
* or continuous) which is from a simulation point actually "outside of time".
*/
class EASEvent {
public:
   EASEvent(const std::string& description) : eventDescription_(description)
   {}

   /**
   * @return  The event's string description.
   */
   std::string getEventDescription() const {
      return eventDescription_;
   }

   /**
   * Sets the event description.
   * 
   * @param description  The description.
   */
   void setEventDescription(const std::string& description) {
      eventDescription_ = description;
   }

private:
   std::string eventDescription_;
};

} // vfm
