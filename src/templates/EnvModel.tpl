@{EnvModel_Header.tpl}@********.include
@{EnvModel_Scaling.tpl}@********.include
@{EnvModel_Parameters.tpl}@********.include -- Needs to be loaded before parameters (like CONCRETE_MODEL) are evaluated below.

@{
@(
@{EnvModel_Geometry.tpl}@********.include

MODULE EnvModel

-- TODO: Taken over from main.tpl. Check if this makes sense or can cause trouble with C++ planner.
INIT node.em.filtered_driver_intention_dir = ActionDir____CENTER;
INIT params.enable_lane_change_driver_request = TRUE;
INIT params.enable_lanechanges = TRUE;
INIT params.slow_lane_is_on_the_right = TRUE;
INIT veh___6TEMPORARRAY19___.v = 0;

INIT !ego.flCond_full;
--INIT !ego.slCond_full;
INIT !ego.abCond_full;
-- EO TODO

@{
## THIS IS JUST AN EXAMPLE OF HOW TO USE THE GEOMETRY FUNCTIONS ##

-- segment l1 and line l2 intersect?
@{@{lines(line(vec(l1.begin.x; l1.begin.y); vec(l1.end.x; l1.end.y)); line(vec(l2.begin.x; l2.begin.y); vec(l2.end.x; l2.end.y)))}@.syntacticSegmentAndLineIntersect}@.serializeNuXmv

-- segment l2 and line l1 intersect?
@{@{lines(line(vec(l2.begin.x; l2.begin.y); vec(l2.end.x; l2.end.y)); line(vec(l1.begin.x; l1.begin.y); vec(l1.end.x; l1.end.y)))}@.syntacticSegmentAndLineIntersect}@.serializeNuXmv

## EO THIS IS JUST AN EXAMPLE OF HOW TO USE THE GEOMETRY FUNCTIONS ##
}@**********.nil


@{
@( -- EM-less build
   @{scalingList}@*.pushBack[-- no,time]

   @{PLANNER_VARIABLES}@.printHeap
)@
@( -- EM-full build

VAR
   cnt : integer;
   num_lanes : integer;

INIT cnt = 0;
TRANS next(cnt) = cnt + 1;

INVAR num_lanes = @{NUMLANES}@.eval[0];


@{EnvModel_Constants.tpl}@********.include
@{EnvModel_Sections.tpl}@*******.include
@{EnvModel_Behavior_Nonego.tpl}@********.include
@{EnvModel_Behavior_Ego.tpl}@********.include
@{EnvModel_Feasibility.tpl}@*******.include

)@
}@**.if[@{EM_LESS}@.eval]
)@
@(
   @{EnvModel_Abstract.tpl}@.include
)@
}@***.if[@{CONCRETE_MODEL}@.eval]
