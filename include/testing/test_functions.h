//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_arith.h"


namespace vfm {
namespace test {

const static std::string SIMPLIFICATION_TEST_BEFORE{ R"(@checkLCConditionsFastLane.agent_index = max(ego.gaps___619___.i_agent_front, 0);
if((env_state.max_lane_id - ego.on_lane >= 2))
{
   @flCond.cond0_lc_conditions_evaluated = true;
   @flCond1LcTriggerDistReached.cond1_lc_trigger_dist_reached = false;
   if((ego.gaps___619___.i_agent_front != -(2)))
   {
      @getLanechangeTriggerDistance.v_lead = veh___609___.v;
      if((0 == 0))
      {
         @getLanechangeTriggerDistance.tau = 2
      }
      else {
         @getLanechangeTriggerDistance.tau = 1
      };
      if((0 == 0))
      {
         @getLanechangeTriggerDistance.delta_tau = 0.20000000298023223877
      }
      else {
         @getLanechangeTriggerDistance.delta_tau = params.lc_trigger_delta_tau_agents
      };
      @getLanechangeTriggerDistance.v_rel = max(ego.v - veh___609___.v, 0);
      @flCond1LcTriggerDistReached.cond1_lc_trigger_dist_reached = (ego.gaps___619___.s_dist_front <= 0) && ((ego.gaps___619___.s_dist_front >= 3) || (0 == 0))
   };
   @flCond.cond1_lc_trigger_dist_reached = flCond1LcTriggerDistReached.cond1_lc_trigger_dist_reached;
   @flCond2EgoFasterThanLead.cond2_ego_faster_than_lead = false;
   if((ego.gaps___619___.i_agent_front != -(2)))
   {
      if((0 == 0))
      {
         @flCond2EgoFasterThanLead.delta = 1
      }
      else {
         @flCond2EgoFasterThanLead.delta = params.lc_trigger_delta_v_agents
      };
      @flCond2EgoFasterThanLead.cond2_ego_faster_than_lead = (ego.v > veh___609___.v + flCond2EgoFasterThanLead.delta)
   };
   @flCond.cond2_ego_faster_than_lead = flCond2EgoFasterThanLead.cond2_ego_faster_than_lead;
   @flCond3LaneSpeedAllowsOvertake.cond3_lane_speed_allows_overtake = false;
   if((ego.gaps___619___.i_agent_front != -(2)))
   {
      @flCond3LaneSpeedAllowsOvertake.v_max_center_lane = min(env_state.v_max_ego, 33.299999237060546875);
      @flCond3LaneSpeedAllowsOvertake.cond3_lane_speed_allows_overtake = (flCond3LaneSpeedAllowsOvertake.v_max_center_lane > veh___609___.v + 1)
   };
   @flCond.cond3_lane_speed_allows_overtake = flCond3LaneSpeedAllowsOvertake.cond3_lane_speed_allows_overtake;
   @flCond4LeadNotMuchFasterThanEgo.cond4_lead_not_much_faster_than_ego = false;
   if((ego.gaps___619___.i_agent_front != -(2)))
   {
      @flCond4LeadNotMuchFasterThanEgo.cond4_lead_not_much_faster_than_ego = (veh___609___.v - ego.v < 1)
   };
   @flCond.cond4_lead_not_much_faster_than_ego = flCond4LeadNotMuchFasterThanEgo.cond4_lead_not_much_faster_than_ego;
   @flCond.cond5_ax_acf_target_lane_same_or_higher = true;
   @flCond6TargetRearVehicleMildBraking.dist_min_fast_approaching = 0;
   if((-(1) != -(2)))
   {
      @flCond6TargetRearVehicleMildBraking.v_rel = 0;
      @TEMPORARRAY1 = -(1);
      if((TEMPORARRAY1 == 1))
      {
         @veh___6TEMPORARRAY19___.v = veh___619___.v
      }
      else {
         if((TEMPORARRAY1 == 0))
         {
            @veh___6TEMPORARRAY19___.v = veh___609___.v
         }
      };
      @flCond6TargetRearVehicleMildBraking.v_rel = max(veh___6TEMPORARRAY19___.v - ego.v, 0);
      @getMinDistanceForFastApproachingVehicles.part1 = flCond6TargetRearVehicleMildBraking.v_rel * params.turn_signal_duration;
      if(false)
      {
         @getMinDistanceForFastApproachingVehicles.ax_approach = 1.1000000238418579102
      }
      else {
         @getMinDistanceForFastApproachingVehicles.ax_approach = 0.60000002384185791016
      };
      @getMinDistanceForFastApproachingVehicles.part2 = flCond6TargetRearVehicleMildBraking.v_rel * flCond6TargetRearVehicleMildBraking.v_rel / getMinDistanceForFastApproachingVehicles.ax_approach * 
      1 / 2;
      if(false)
      {
         @getMinDistanceForFastApproachingVehicles.tau_min = params.cond_tau_min_rear_static_abort
      }
      else {
         @getMinDistanceForFastApproachingVehicles.tau_min = 0.40000000596046447754
      };
      @flCond6TargetRearVehicleMildBraking.dist_min_fast_approaching = getMinDistanceForFastApproachingVehicles.part1 + getMinDistanceForFastApproachingVehicles.part2 + 5
   };
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___629___.s_dist_rear
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___619___.s_dist_rear
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___609___.s_dist_rear
         }
      }
   };
   @flCond.cond6_target_rear_vehicle_mild_braking = (ego.gaps___6tar_dir9___.s_dist_rear >= flCond6TargetRearVehicleMildBraking.dist_min_fast_approaching);
   if(false)
   {
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front = max(5, ego.v * 0.5)
   }
   else {
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front = max(5, ego.v * 1.2000000476837158203)
   };
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___629___.i_agent_front
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___619___.i_agent_front
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___609___.i_agent_front
         }
      }
   };
   @flCond7DistToTargetFrontVehicleLargeEnough.i_agent_front = ego.gaps___6tar_dir9___.i_agent_front;
   if((flCond7DistToTargetFrontVehicleLargeEnough.i_agent_front != -(2)))
   {
      if((ActionDir::NONE == tar_dir))
      {
         @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___629___.i_agent_front
      }
      else {
         if((ActionDir::RIGHT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___619___.i_agent_front
         }
         else {
            if((ActionDir::LEFT == tar_dir))
            {
               @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___609___.i_agent_front
            }
         }
      };
      @TEMPORARRAY2 = ego.gaps___6tar_dir9___.i_agent_front;
      if((TEMPORARRAY2 == 1))
      {
         @veh___6TEMPORARRAY29___.v = veh___619___.v
      }
      else {
         if((TEMPORARRAY2 == 0))
         {
            @veh___6TEMPORARRAY29___.v = veh___609___.v
         }
      };
      @flCond7DistToTargetFrontVehicleLargeEnough.delta_v = max(veh___6TEMPORARRAY29___.v - ego.v, 0);
      @flCond7DistToTargetFrontVehicleLargeEnough.delta_dist = flCond7DistToTargetFrontVehicleLargeEnough.delta_v * params.turn_signal_duration;
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front = max(flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front - flCond7DistToTargetFrontVehicleLargeEnough.delta_dist, 
         0);
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front_dyn = 0;
      @flCond7DistToTargetFrontVehicleLargeEnough.v_rel_tmp = 0;
      if((ActionDir::NONE == tar_dir))
      {
         @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___629___.i_agent_front
      }
      else {
         if((ActionDir::RIGHT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___619___.i_agent_front
         }
         else {
            if((ActionDir::LEFT == tar_dir))
            {
               @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___609___.i_agent_front
            }
         }
      };
      @TEMPORARRAY3 = ego.gaps___6tar_dir9___.i_agent_front;
      if((TEMPORARRAY3 == 1))
      {
         @veh___6TEMPORARRAY39___.v = veh___619___.v
      }
      else {
         if((TEMPORARRAY3 == 0))
         {
            @veh___6TEMPORARRAY39___.v = veh___609___.v
         }
      };
      @flCond7DistToTargetFrontVehicleLargeEnough.v_rel_tmp = max(ego.v - veh___6TEMPORARRAY39___.v, 0);
      @getDynamicMinDistanceFront.part1 = flCond7DistToTargetFrontVehicleLargeEnough.v_rel_tmp * flCond7DistToTargetFrontVehicleLargeEnough.v_rel_tmp / 0.5 * 1 / 2;
      @getDynamicMinDistanceFront.part2 = 0;
      if(false)
      {
         @getDynamicMinDistanceFront.part2 = ego.v * 0.5
      }
      else {
         @getDynamicMinDistanceFront.part2 = ego.v * 1.2000000476837158203
      };
      @getDynamicMinDistanceFront.part2 = max(getDynamicMinDistanceFront.part2, 5);
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front_dyn = getDynamicMinDistanceFront.part1 + getDynamicMinDistanceFront.part2;
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front = max(flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front, 
         flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front_dyn)
   };
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.s_dist_front = ego.gaps___629___.s_dist_front
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.s_dist_front = ego.gaps___619___.s_dist_front
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.s_dist_front = ego.gaps___609___.s_dist_front
         }
      }
   };
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___629___.i_agent_front
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___619___.i_agent_front
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___609___.i_agent_front
         }
      }
   };
   @flCond7DistToTargetFrontVehicleLargeEnough.cond7_dist_to_target_front_vehicle_large_enough = (ego.gaps___6tar_dir9___.s_dist_front > 
   flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front) || (ego.gaps___6tar_dir9___.i_agent_front == -(2));
   @flCond.cond7_dist_to_target_front_vehicle_large_enough = flCond7DistToTargetFrontVehicleLargeEnough.cond7_dist_to_target_front_vehicle_large_enough;
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___629___.s_dist_rear
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___619___.s_dist_rear
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___609___.s_dist_rear
         }
      }
   };
   @flCond.cond8_dist_to_target_rear_vehicle_large_enough = (ego.gaps___6tar_dir9___.s_dist_rear > 5) || (-(1) == -(2));
   @flCond.cond10_t_action_ok = (ego.time_since_last_lc >= 2);
   if((0 == 0))
   {
      @flCond.cond11_enable_lanechanges = params.enable_lanechanges
   }
   else {
      @flCond.cond11_enable_lanechanges = params.enable_agent_lanechange_prediction
   };
   if(params.slow_lane_is_on_the_right)
   {
      @flCond.cond12_current_lane_is_merge_lane = true
   }
   else {
      @flCond.cond12_current_lane_is_merge_lane = true
   };
   @flCond.cond14_shadow_agent_ahead = false;
   @flCond.cond16_forced_merge = false;
   @flCond.cond18_driver_lanechange_request = (0 == 0) && ((node.em.filtered_driver_intention_dir == ActionDir::LEFT) && params.slow_lane_is_on_the_right || (node.em.filtered_driver_intention_dir == 
   ActionDir::RIGHT) && !(params.slow_lane_is_on_the_right));
   @flCond.cond19_navigation_system_request = false;
   @checkLCConditionsFastLane.cond20_reactive_motivation = flCond.cond1_lc_trigger_dist_reached && (flCond.cond2_ego_faster_than_lead || flCond.cond3_lane_speed_allows_overtake && 
   flCond.cond4_lead_not_much_faster_than_ego && (0 == 0)) && flCond.cond5_ax_acf_target_lane_same_or_higher || flCond.cond14_shadow_agent_ahead || flCond.cond16_forced_merge || 
   ego.slow_lane_deadlock_active;
   @checkLCConditionsFastLane.cond20_reactive_timing = flCond.cond10_t_action_ok || flCond.cond14_shadow_agent_ahead || flCond.cond16_forced_merge;
   @checkLCConditionsFastLane.cond20_reactive_enablers = flCond.cond11_enable_lanechanges;
   @flCond.cond20_reactive_conditions_fulfilled = checkLCConditionsFastLane.cond20_reactive_motivation && checkLCConditionsFastLane.cond20_reactive_timing && 
   checkLCConditionsFastLane.cond20_reactive_enablers;
   @checkLCConditionsFastLane.cond22_safety_dynamic = flCond.cond6_target_rear_vehicle_mild_braking;
   @checkLCConditionsFastLane.cond22_safety_static = flCond.cond7_dist_to_target_front_vehicle_large_enough && flCond.cond8_dist_to_target_rear_vehicle_large_enough;
   @flCond.cond22_safety_conditions_fulfilled = checkLCConditionsFastLane.cond22_safety_dynamic && checkLCConditionsFastLane.cond22_safety_static;
   @flCond.cond24_external_conditions_fulfilled = flCond.cond18_driver_lanechange_request && params.enable_lane_change_driver_request || flCond.cond19_navigation_system_request;
   @flCond.cond26_all_conditions_fulfilled_raw = (flCond.cond20_reactive_conditions_fulfilled || flCond.cond24_external_conditions_fulfilled) && flCond.cond22_safety_conditions_fulfilled
}
)" };

const static std::string SIMPLIFICATION_TEST_AFTER{ R"(@checkLCConditionsFastLane.agent_index = max(ego.gaps___619___.i_agent_front, 0);
if((env_state.max_lane_id - ego.on_lane >= 2))
{
   @flCond.cond0_lc_conditions_evaluated = true;
   @flCond1LcTriggerDistReached.cond1_lc_trigger_dist_reached = false;
   if((ego.gaps___619___.i_agent_front != -(2)))
   {
      @flCond1LcTriggerDistReached.cond1_lc_trigger_dist_reached = (ego.gaps___619___.s_dist_front <= 0);
      @getLanechangeTriggerDistance.delta_tau = 0.20000000298023223877;
      @getLanechangeTriggerDistance.tau = 2;
      @getLanechangeTriggerDistance.v_lead = veh___609___.v;
      @getLanechangeTriggerDistance.v_rel = max(ego.v - veh___609___.v, 0)
   };
   @flCond.cond1_lc_trigger_dist_reached = flCond1LcTriggerDistReached.cond1_lc_trigger_dist_reached;
   @flCond2EgoFasterThanLead.cond2_ego_faster_than_lead = false;
   if((ego.gaps___619___.i_agent_front != -(2)))
   {
      @flCond2EgoFasterThanLead.cond2_ego_faster_than_lead = (ego.v > 1 + veh___609___.v);
      @flCond2EgoFasterThanLead.delta = 1
   };
   @flCond.cond2_ego_faster_than_lead = flCond2EgoFasterThanLead.cond2_ego_faster_than_lead;
   @flCond3LaneSpeedAllowsOvertake.cond3_lane_speed_allows_overtake = false;
   if((ego.gaps___619___.i_agent_front != -(2)))
   {
      @flCond3LaneSpeedAllowsOvertake.cond3_lane_speed_allows_overtake = (min(env_state.v_max_ego, 33.299999237060546875) > 1 + veh___609___.v);
      @flCond3LaneSpeedAllowsOvertake.v_max_center_lane = min(env_state.v_max_ego, 33.299999237060546875)
   };
   @flCond.cond3_lane_speed_allows_overtake = flCond3LaneSpeedAllowsOvertake.cond3_lane_speed_allows_overtake;
   @flCond4LeadNotMuchFasterThanEgo.cond4_lead_not_much_faster_than_ego = false;
   if((ego.gaps___619___.i_agent_front != -(2)))
   {
      @flCond4LeadNotMuchFasterThanEgo.cond4_lead_not_much_faster_than_ego = (veh___609___.v - ego.v < 1)
   };
   @flCond.cond4_lead_not_much_faster_than_ego = flCond4LeadNotMuchFasterThanEgo.cond4_lead_not_much_faster_than_ego;
   @flCond.cond5_ax_acf_target_lane_same_or_higher = true;
   @flCond6TargetRearVehicleMildBraking.dist_min_fast_approaching = 0;
   @flCond6TargetRearVehicleMildBraking.v_rel = 0;
   @TEMPORARRAY1 = -(1);
   @flCond6TargetRearVehicleMildBraking.v_rel = max(veh___6TEMPORARRAY19___.v - ego.v, 0);
   @getMinDistanceForFastApproachingVehicles.part1 = max(veh___6TEMPORARRAY19___.v - ego.v, 0) * params.turn_signal_duration;
   if(false)
   {
      @getMinDistanceForFastApproachingVehicles.ax_approach = 1.1000000238418579102
   }
   else {
      @getMinDistanceForFastApproachingVehicles.ax_approach = 0.60000002384185791016
   };
   @getMinDistanceForFastApproachingVehicles.part2 = 0.5 * flCond6TargetRearVehicleMildBraking.v_rel * flCond6TargetRearVehicleMildBraking.v_rel / getMinDistanceForFastApproachingVehicles.ax_approach;
   if(false)
   {
      @getMinDistanceForFastApproachingVehicles.tau_min = params.cond_tau_min_rear_static_abort
   }
   else {
      @getMinDistanceForFastApproachingVehicles.tau_min = 0.40000000596046447754
   };
   @flCond6TargetRearVehicleMildBraking.dist_min_fast_approaching = 5 + getMinDistanceForFastApproachingVehicles.part1 + getMinDistanceForFastApproachingVehicles.part2;
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___629___.s_dist_rear
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___619___.s_dist_rear
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___609___.s_dist_rear
         }
      }
   };
   @flCond.cond6_target_rear_vehicle_mild_braking = (ego.gaps___6tar_dir9___.s_dist_rear >= flCond6TargetRearVehicleMildBraking.dist_min_fast_approaching);
   if(false)
   {
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front = max(5, 0.5 * ego.v)
   }
   else {
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front = max(5, 1.2000000476837158203 * ego.v)
   };
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___629___.i_agent_front
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___619___.i_agent_front
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___609___.i_agent_front
         }
      }
   };
   @flCond7DistToTargetFrontVehicleLargeEnough.i_agent_front = ego.gaps___6tar_dir9___.i_agent_front;
   if((flCond7DistToTargetFrontVehicleLargeEnough.i_agent_front != -(2)))
   {
      if((ActionDir::NONE == tar_dir))
      {
         @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___629___.i_agent_front
      }
      else {
         if((ActionDir::RIGHT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___619___.i_agent_front
         }
         else {
            if((ActionDir::LEFT == tar_dir))
            {
               @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___609___.i_agent_front
            }
         }
      };
      @TEMPORARRAY2 = ego.gaps___6tar_dir9___.i_agent_front;
      if((ego.gaps___6tar_dir9___.i_agent_front == 1))
      {
         @veh___6TEMPORARRAY29___.v = veh___619___.v
      }
      else {
         if((ego.gaps___6tar_dir9___.i_agent_front == 0))
         {
            @veh___6TEMPORARRAY29___.v = veh___609___.v
         }
      };
      @flCond7DistToTargetFrontVehicleLargeEnough.delta_dist = max(veh___6TEMPORARRAY29___.v - ego.v, 0) * params.turn_signal_duration;
      @flCond7DistToTargetFrontVehicleLargeEnough.delta_v = max(veh___6TEMPORARRAY29___.v - ego.v, 0);
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front = max(flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front - max(veh___6TEMPORARRAY29___.v - ego.v, 0) *
      params.turn_signal_duration, 0);
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front_dyn = 0;
      @flCond7DistToTargetFrontVehicleLargeEnough.v_rel_tmp = 0;
      if((ActionDir::NONE == tar_dir))
      {
         @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___629___.i_agent_front
      }
      else {
         if((ActionDir::RIGHT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___619___.i_agent_front
         }
         else {
            if((ActionDir::LEFT == tar_dir))
            {
               @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___609___.i_agent_front
            }
         }
      };
      @TEMPORARRAY3 = ego.gaps___6tar_dir9___.i_agent_front;
      if((ego.gaps___6tar_dir9___.i_agent_front == 1))
      {
         @veh___6TEMPORARRAY39___.v = veh___619___.v
      }
      else {
         if((ego.gaps___6tar_dir9___.i_agent_front == 0))
         {
            @veh___6TEMPORARRAY39___.v = veh___609___.v
         }
      };
      @flCond7DistToTargetFrontVehicleLargeEnough.v_rel_tmp = max(ego.v - veh___6TEMPORARRAY39___.v, 0);
      @getDynamicMinDistanceFront.part1 = 0.5 * max(ego.v - veh___6TEMPORARRAY39___.v, 0) * max(ego.v - veh___6TEMPORARRAY39___.v, 0) / 0.5;
      @getDynamicMinDistanceFront.part2 = 0;
      if(false)
      {
         @getDynamicMinDistanceFront.part2 = 0.5 * ego.v
      }
      else {
         @getDynamicMinDistanceFront.part2 = 1.2000000476837158203 * ego.v
      };
      @getDynamicMinDistanceFront.part2 = max(getDynamicMinDistanceFront.part2, 5);
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front = max(flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front, getDynamicMinDistanceFront.part1 +
         getDynamicMinDistanceFront.part2);
      @flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front_dyn = getDynamicMinDistanceFront.part1 + getDynamicMinDistanceFront.part2
   };
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.s_dist_front = ego.gaps___629___.s_dist_front
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.s_dist_front = ego.gaps___619___.s_dist_front
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.s_dist_front = ego.gaps___609___.s_dist_front
         }
      }
   };
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___629___.i_agent_front
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___619___.i_agent_front
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.i_agent_front = ego.gaps___609___.i_agent_front
         }
      }
   };
   @flCond7DistToTargetFrontVehicleLargeEnough.cond7_dist_to_target_front_vehicle_large_enough = (ego.gaps___6tar_dir9___.s_dist_front >
   flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front) || (ego.gaps___6tar_dir9___.i_agent_front == -(2));
   @flCond.cond7_dist_to_target_front_vehicle_large_enough = (ego.gaps___6tar_dir9___.s_dist_front > flCond7DistToTargetFrontVehicleLargeEnough.min_dist_target_front) ||
   (ego.gaps___6tar_dir9___.i_agent_front == -(2));
   if((ActionDir::NONE == tar_dir))
   {
      @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___629___.s_dist_rear
   }
   else {
      if((ActionDir::RIGHT == tar_dir))
      {
         @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___619___.s_dist_rear
      }
      else {
         if((ActionDir::LEFT == tar_dir))
         {
            @ego.gaps___6tar_dir9___.s_dist_rear = ego.gaps___609___.s_dist_rear
         }
      }
   };
   @checkLCConditionsFastLane.cond20_reactive_enablers = params.enable_lanechanges;
   @checkLCConditionsFastLane.cond20_reactive_motivation = flCond.cond1_lc_trigger_dist_reached && (flCond.cond2_ego_faster_than_lead || flCond.cond3_lane_speed_allows_overtake &&
   flCond.cond4_lead_not_much_faster_than_ego) && flCond.cond5_ax_acf_target_lane_same_or_higher || ego.slow_lane_deadlock_active || false || false;
   @checkLCConditionsFastLane.cond20_reactive_timing = (ego.time_since_last_lc >= 2) || false || false;
   @checkLCConditionsFastLane.cond22_safety_dynamic = flCond.cond6_target_rear_vehicle_mild_braking;
   @checkLCConditionsFastLane.cond22_safety_static = flCond.cond7_dist_to_target_front_vehicle_large_enough && (ego.gaps___6tar_dir9___.s_dist_rear > 5);
   @flCond.cond10_t_action_ok = (ego.time_since_last_lc >= 2);
   @flCond.cond11_enable_lanechanges = params.enable_lanechanges;
   @flCond.cond12_current_lane_is_merge_lane = true;
   @flCond.cond14_shadow_agent_ahead = false;
   @flCond.cond16_forced_merge = false;
   @flCond.cond18_driver_lanechange_request = (ActionDir::LEFT == node.em.filtered_driver_intention_dir) && params.slow_lane_is_on_the_right || (ActionDir::RIGHT ==
   node.em.filtered_driver_intention_dir) && !(params.slow_lane_is_on_the_right);
   @flCond.cond19_navigation_system_request = false;
   @flCond.cond20_reactive_conditions_fulfilled = params.enable_lanechanges && (flCond.cond1_lc_trigger_dist_reached && (flCond.cond2_ego_faster_than_lead || flCond.cond3_lane_speed_allows_overtake
   && flCond.cond4_lead_not_much_faster_than_ego) && flCond.cond5_ax_acf_target_lane_same_or_higher || ego.slow_lane_deadlock_active || false || false) && ((ego.time_since_last_lc >= 2) || false ||
   false);
   @flCond.cond22_safety_conditions_fulfilled = flCond.cond6_target_rear_vehicle_mild_braking && flCond.cond7_dist_to_target_front_vehicle_large_enough && (ego.gaps___6tar_dir9___.s_dist_rear > 5);
   @flCond.cond24_external_conditions_fulfilled = ((ActionDir::LEFT == node.em.filtered_driver_intention_dir) && params.slow_lane_is_on_the_right || (ActionDir::RIGHT ==
   node.em.filtered_driver_intention_dir) && !(params.slow_lane_is_on_the_right)) && params.enable_lane_change_driver_request || false;
   @flCond.cond26_all_conditions_fulfilled_raw = (params.enable_lanechanges && (flCond.cond1_lc_trigger_dist_reached && (flCond.cond2_ego_faster_than_lead || flCond.cond3_lane_speed_allows_overtake
   && flCond.cond4_lead_not_much_faster_than_ego) && flCond.cond5_ax_acf_target_lane_same_or_higher || ego.slow_lane_deadlock_active || false || false) && ((ego.time_since_last_lc >= 2) || false ||
   false) || ((ActionDir::LEFT == node.em.filtered_driver_intention_dir) && params.slow_lane_is_on_the_right || (ActionDir::RIGHT == node.em.filtered_driver_intention_dir) &&
   !(params.slow_lane_is_on_the_right)) && params.enable_lane_change_driver_request || false) && flCond.cond6_target_rear_vehicle_mild_braking &&
   flCond.cond7_dist_to_target_front_vehicle_large_enough && (ego.gaps___6tar_dir9___.s_dist_rear > 5);
   @flCond.cond8_dist_to_target_rear_vehicle_large_enough = (ego.gaps___6tar_dir9___.s_dist_rear > 5)
})" };

std::map<std::string, bool> runTests(const int from = -1, const int to = -1);

} // test
} // vfm