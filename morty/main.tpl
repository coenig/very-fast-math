#include "planner.cpp_combined.k2.smv"
#include "EnvModel.smv"


MODULE Globals
VAR
"loc" : boolean;

MODULE main
VAR
  globals : Globals; 
  env : EnvModel;
  planner : "checkLCConditionsFastLane"(globals);
                   

   lane_changed : boolean;
   lane_change_aborted : boolean;


-- TRANS env.abCond.cond26_all_conditions_fulfilled_raw = planner."abCond.cond26_all_conditions_fulfilled_raw";
-- TRANS env.flCond.cond26_all_conditions_fulfilled_raw = planner."flCond.cond26_all_conditions_fulfilled_raw";


--TRANS env.ego.abCond_full = ((env.ego_timer > 0) & !planner."agent.slCond_full" & !planner."agent.flCond_full");

--DEFINE
--env.ego.flCond_full := planner."flCond.cond26_all_conditions_fulfilled_raw";
--env.ego.abCond_full := planner."abCond.cond26_all_conditions_fulfilled_raw";

--TRANS next(env.ego.flCond_full) = planner."flCond.cond26_all_conditions_fulfilled_raw";
--TRANS next(env.ego.abCond_full) = planner."abCond.cond26_all_conditions_fulfilled_raw";


-- Specs follow here
DEFINE



--INVARSPEC cnt < 10 | abs(env.veh___609___.rel_pos) >= 10 | abs(env.veh___619___.rel_pos) >= 10 | abs(env.veh___629___.rel_pos) >= 10;
--INVARSPEC env.ego_timer < 1;

--INVARSPEC env.ego_timer <= next(env.ego_timer) | aborted;
--CTLSPEC AG(cnt = 0);

--INVARSPEC !planner."abCond.cond26_all_conditions_fulfilled_raw";
--INVARSPEC !lane_change_aborted | env.ego_lane_crossing;
--INVARSPEC !env.blamable_crash;

--INVARSPEC !env.ego_lane_b4;
--INVARSPEC env.ego.abs_pos >= env.segment_1_pos_begin & env.ego.abs_pos < env.segment_2_pos_begin & env.segment_1_min_lane > 0 -> !env.ego_lane_b0;

--SPEC-STUFF
INVARSPEC env.veh___619___.v > 0;
--EO-SPEC-STUFF
