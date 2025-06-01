#include "@{FILENAME_PLANNER}@.printHeap"
#include "@{FILENAME_ENVMODEL}@.printHeap"


MODULE Globals
VAR
"loc" : boolean;

MODULE main
VAR
  globals : Globals; 
  env : EnvModel;
  planner : "@{PLANNER_ENTRY_FILENAME}@.printHeap"(globals@{PLANNER_PARAMETERS}@.printHeap);
                   
@{
@(

cnt : integer;
INIT cnt = 0;
TRANS next(cnt) = cnt + 1;

TRANS next(env.input.m_gpFunktionsstatusGetriggerterFsw.m_value) = planner."gpFunktionsstatusGetriggerterFsw.m_value";
-- TRANS next(env.input.m_gpFunktionsstatusGetriggerterFsw.m_valueLastCycle) = planner."gpFunktionsstatusGetriggerterFsw.m_valueLastCycle";

INVARSPEC
     env.input.m_gpFunktionsstatusGetriggerterFsw.m_value = Value____NichtVerfuegbar | env.input.m_gpFunktionsstatusGetriggerterFsw.m_value = Value____Verfuegbar;

-- INVARSPEC 
--    ((env.input.m_gpFunktionsstatusGetriggerterFsw.m_value != Value____AktivRegeltLinks)
--      & (env.input.m_gpFunktionsstatusGetriggerterFsw.m_value != Value____AktivWunschErkanntLinks)
--      & (env.input.m_gpFunktionsstatusGetriggerterFsw.m_value != Value____AktivWartetLinks)) -> 
--      (planner."gpFunktionsstatusGetriggerterFsw.m_value" != Value____AktivRegeltLinks);

-- INVARSPEC
--     (planner."gpFunktionsstatusGetriggerterFsw.m_value")
--     ->
--      (
--       (env.input.m_gpFunktionsstatusGetriggerterFsw.m_value = Value____AktivRegeltLinks) |
--       (env.input.m_gpFunktionsstatusGetriggerterFsw.m_value = Value____AktivWunschErkanntLinks) |
--       (env.input.m_gpFunktionsstatusGetriggerterFsw.m_value = Value____AktivWartetLinks)
--      );
)@
@(
   lane_changed : boolean;
   lane_change_aborted : boolean;

   env.node.em.filtered_driver_intention_dir : {ActionDir____LEFT, ActionDir____CENTER, ActionDir____RIGHT};
   env.params.enable_lane_change_driver_request : boolean;
   env.params.enable_lanechanges : boolean;
   env.params.slow_lane_is_on_the_right : boolean;
   env.veh___6TEMPORARRAY19___.v : integer;

INIT env.node.em.filtered_driver_intention_dir = ActionDir____CENTER;
INIT env.params.enable_lane_change_driver_request = TRUE;
INIT env.params.enable_lanechanges = TRUE;
INIT env.params.slow_lane_is_on_the_right = TRUE;
INIT env.veh___6TEMPORARRAY19___.v = 0;

INIT !env.ego.flCond_full;
--INIT !env.ego.slCond_full;
INIT !env.ego.abCond_full;
INIT !lane_changed;
INIT !lane_change_aborted;

@{TRANSITIONS_PLANNER_TO_ENVMODEL}@.printHeap

--TRANS env.ego.abCond_full = ((env.ego_timer > 0) & !planner."agent.slCond_full" & !planner."agent.flCond_full");
TRANS next(lane_changed) = (lane_changed | planner."flCond.cond26_all_conditions_fulfilled_raw");
@{
TRANS next(lane_change_aborted) = ((lane_change_aborted | planner."abCond.cond26_all_conditions_fulfilled_raw" & env.ego_lane_crossing));
}@.if[!@{EGOLESS}@.eval]

--DEFINE
--env.ego.flCond_full := planner."flCond.cond26_all_conditions_fulfilled_raw";
--env.ego.abCond_full := planner."abCond.cond26_all_conditions_fulfilled_raw";

--TRANS next(env.ego.flCond_full) = planner."flCond.cond26_all_conditions_fulfilled_raw";
--TRANS next(env.ego.abCond_full) = planner."abCond.cond26_all_conditions_fulfilled_raw";
TRANS env.ego.flCond_full = planner."flCond.cond26_all_conditions_fulfilled_raw";
TRANS env.ego.abCond_full = planner."abCond.cond26_all_conditions_fulfilled_raw";


-- Specs follow here
DEFINE
--SPEC-STUFF
@{BB1}@.printHeap
@{BB2}@.printHeap
@{BB3}@.printHeap

@{SPEC}@.printHeap -- Specification from envmodel_config.json.
--EO-SPEC-STUFF


--INVARSPEC cnt < 10 | abs(env.veh___609___.rel_pos) >= 10 | abs(env.veh___619___.rel_pos) >= 10 | abs(env.veh___629___.rel_pos) >= 10;
--INVARSPEC env.ego_timer < 1;

--INVARSPEC env.ego_timer <= next(env.ego_timer) | aborted;
--CTLSPEC AG(cnt = 0);

--INVARSPEC !planner."abCond.cond26_all_conditions_fulfilled_raw";
--INVARSPEC !lane_change_aborted | env.ego_lane_crossing;
--INVARSPEC !env.blamable_crash;

--INVARSPEC !env.ego_lane_b4;
--INVARSPEC env.ego.abs_pos >= env.segment_1_pos_begin & env.ego.abs_pos < env.segment_2_pos_begin & env.segment_1_min_lane > 0 -> !env.ego_lane_b0;
)@
}@*.if[@{EM_LESS}@.eval]