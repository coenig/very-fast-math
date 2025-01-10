//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include <array>
#include <iostream>

// #vfm-insert-includes

// #vfm-begin
// #vfm-insert-associativity-pretext

enum class ActionDir
{
    LEFT = 0,
    CENTER = 1,
    RIGHT = 2,
    NONE = 3
};

struct Gap
{
   int i_agent_front{-1};    // #vfm-tal[[-1..2]]
   int s_dist_front{256};    // #vfm-tal[[0..256]]
   int v_front{70};          // #vfm-tal[[0..70]]
   int a_front{0};           // #vfm-tal[[-8..6]]

   int lane_availability{0}; // #vfm-tal[[0..256]]  // Position ahead where lane disappears (up to MAX_VISIBILITY_RANGE)

   int i_agent_rear{-1};     // #vfm-tal[[-1..2]]
   int s_dist_rear{256};     // #vfm-tal[[0..256]]
   int v_rear{0};            // #vfm-tal[[0..70]]

   ActionDir turn_signals_front{ActionDir::NONE}; // Enum needs no TAL (type abstraction layer) information.

   static constexpr int i_FREE_LANE = -1;
   
   // structs / classes cannot contain member functions, but can use inheritance.
};

struct Agent {
   int v{}; // #vfm-tal[[0..34]]
   int a{}; // #vfm-tal[[-8..6]]
   bool has_close_vehicle_on_left_left_lane{}; // bool needs no TAL information.
   bool has_close_vehicle_on_right_right_lane{};
   bool flCond_full{}; // Lane change towards the "fast lane" (fl), i.e., left lane in our case.
   bool slCond_full{}; // Lane change towards the "slow lane" (sl), i.e., right lane in our case.
   std::array<Gap, 3> gaps;
};

// Entrance function for Model Checking.
// Note that (for the time being), atomic variables are categorized based on their usage.
// Therefore, variables considered as
//  - input from the EnvModel may not be written, and
//  - output to the EnvModel may not be read.
// Otherwise they would be wrongly categorized as
//  - internal variables without an interface to the EnvModel.
// This does not apply to a full struct but only to the actual primitive variables of the struct.
// E.g., below
//  - agent.v is an input variable which is never written to;
//  - agent.a is an output variable which is never read from.
void plan(
   Agent& agent // #vfm-aka[[ literal(ego) ]]    // Variable mapping planner <-> env model: For illustration prupose, let's name ego "agent" and map it onto variable "ego" in the env model.
   )                                             // Note that the individual fields (agent.v etc.) can be "aka"d, too, though currently only if the base "agent" is not "aka"d.
{
   // #vfm-gencode-begin[[ condition=false ]]
   static constexpr int MIN_DIST = 14;
   
   // Lane Changes Left and Right
   agent.flCond_full = false; // Fast lane condition.
   agent.slCond_full = false; // Slow lane condition.
   
   if (agent.gaps[static_cast<int>(ActionDir::LEFT)].lane_availability > MIN_DIST          // LEFT lane is available.
       && (agent.gaps[static_cast<int>(ActionDir::CENTER)].s_dist_front < MIN_DIST         //   In front of us is...
       && agent.v > agent.gaps[static_cast<int>(ActionDir::CENTER)].v_front                //   ...a dawdler.
       && agent.v < agent.gaps[static_cast<int>(ActionDir::LEFT)].v_front                  //   The front guy on the LEFT is faster than us.
          || agent.gaps[static_cast<int>(ActionDir::CENTER)].lane_availability < 150) //   OR our lane is disappearing
       && agent.gaps[static_cast<int>(ActionDir::LEFT)].s_dist_rear > MIN_DIST             // There's a gap...
       && agent.gaps[static_cast<int>(ActionDir::LEFT)].s_dist_front > MIN_DIST            // ...large enough to fit in to the LEFT.
       // && !agent.has_close_vehicle_on_left_left_lane
       ) 
   {
      agent.flCond_full = true; 
   } else if (agent.gaps[static_cast<int>(ActionDir::RIGHT)].lane_availability > MIN_DIST         // RIGHT lane is available.
              && (agent.v < agent.gaps[static_cast<int>(ActionDir::RIGHT)].v_front                //   The front guy on the RIGHT is faster than us.
                 || agent.gaps[static_cast<int>(ActionDir::CENTER)].lane_availability < 150) //   OR our lane is disappearing
              && agent.gaps[static_cast<int>(ActionDir::RIGHT)].s_dist_front > MIN_DIST           // There's a gap...
              && agent.gaps[static_cast<int>(ActionDir::RIGHT)].s_dist_rear > MIN_DIST            // ...large enough to fit in to the RIGHT.
              // && !agent.has_close_vehicle_on_right_right_lane                          
              )
   {
      agent.slCond_full = true;
   }

   // Acceleration
   bool ego_pressured_from_ahead_on_left_lane = agent.gaps[static_cast<int>(ActionDir::LEFT)].turn_signals_front == ActionDir::RIGHT;
   bool ego_pressured_from_ahead_on_right_lane = agent.gaps[static_cast<int>(ActionDir::RIGHT)].turn_signals_front == ActionDir::LEFT;
   int ego_following_dist = std::max(2/*0*/ * agent.v, 5); // #vfm-tal[[5..78]]
   int allowed_ego_a_front_center = std::max(std::min(agent.gaps[static_cast<int>(ActionDir::CENTER)].s_dist_front + agent.gaps[static_cast<int>(ActionDir::CENTER)].v_front + agent.gaps[static_cast<int>(ActionDir::CENTER)].a_front - agent.v - ego_following_dist, 2), -8);
   int allowed_ego_a_front_right = std::max(std::min(agent.gaps[static_cast<int>(ActionDir::RIGHT)].s_dist_front + agent.gaps[static_cast<int>(ActionDir::RIGHT)].v_front + agent.gaps[static_cast<int>(ActionDir::RIGHT)].a_front - agent.v - ego_following_dist, 2), -8);
   int allowed_ego_a_front_left = std::max(std::min(agent.gaps[static_cast<int>(ActionDir::LEFT)].s_dist_front + agent.gaps[static_cast<int>(ActionDir::LEFT)].v_front + agent.gaps[static_cast<int>(ActionDir::LEFT)].a_front - agent.v - ego_following_dist, 2), -8);
   int acceleration = std::min(allowed_ego_a_front_center, 2);

   if (agent.gaps[static_cast<int>(ActionDir::CENTER)].s_dist_front < 10
        || (agent.gaps[static_cast<int>(ActionDir::LEFT)].s_dist_front < 10 && ego_pressured_from_ahead_on_left_lane)
        || (agent.gaps[static_cast<int>(ActionDir::RIGHT)].s_dist_front < 10 && ego_pressured_from_ahead_on_right_lane)
        || agent.gaps[static_cast<int>(ActionDir::CENTER)].lane_availability < 40) {
        acceleration = std::max(-agent.v, -8);
   } else {
        if (ego_pressured_from_ahead_on_right_lane) {
           acceleration = std::min(acceleration, allowed_ego_a_front_right);
        }
        if (ego_pressured_from_ahead_on_left_lane) {
           acceleration = std::min(acceleration, allowed_ego_a_front_left);
        }
   }
   
   agent.a = acceleration;
   // #vfm-gencode-end
} 
// #vfm-end

int main()
{
   std::array<Gap, 3> dummy_gaps{};
   Agent dummy_ego{ 0, 0, false, false, false, false, dummy_gaps };
   int iteration{};
   
   for (;;) {
      dummy_ego.v = std::min(std::max(dummy_ego.v + dummy_ego.a, 0), 34);
      plan(dummy_ego);
      std::cout << "Iteration " << iteration++ << ": ("
               << "ego.v = " << dummy_ego.v << "; "
               << "ego.a = " << dummy_ego.a << "; "
               << "ego.flCond_full = " << dummy_ego.flCond_full << "; "
               << "ego.slCond_full = " << dummy_ego.slCond_full << ""
               << ")" << std::endl;
   }
   
   return 0;
}
