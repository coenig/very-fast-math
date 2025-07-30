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
}@.if[@{!EGOLESS}@.eval]

--DEFINE
--env.ego.flCond_full := planner."flCond.cond26_all_conditions_fulfilled_raw";
--env.ego.abCond_full := planner."abCond.cond26_all_conditions_fulfilled_raw";

--TRANS next(env.ego.flCond_full) = planner."flCond.cond26_all_conditions_fulfilled_raw";
--TRANS next(env.ego.abCond_full) = planner."abCond.cond26_all_conditions_fulfilled_raw";
TRANS env.ego.flCond_full = planner."flCond.cond26_all_conditions_fulfilled_raw";
TRANS env.ego.abCond_full = planner."abCond.cond26_all_conditions_fulfilled_raw";


--SPEC-STUFF
-- Don't change the wording of the above line and its corresponding closing line! It's used to detect the SPEC part
-- to be able to replace just it when running the MC without re-generating the EnvModel. It's also used for UCD.

@{
@{
@{SPEC}@.printHeap
@{
   @{@{SPEC[i]}@.printHeap}@*.if[@{SPEC[i]}@.vfm_variable_declared]
}@**.for[[i], 0, 100]
}@.@{@(replaceAll[LTLSPEC, @"{LTLSPEC NAME }"@spec$$$$$@"{ :=}"@])@@(id)@}@*.if[@{SCENGEN_MODE}@.eval]
}@.replaceAllCounting[$$$$$]

--EO-SPEC-STUFF
)@
}@*.if[@{EM_LESS}@.eval]