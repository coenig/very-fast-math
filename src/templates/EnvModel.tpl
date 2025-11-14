@{
@(
@{EnvModel_Header.tpl}@********.include
@{EnvModel_Scaling.tpl}@********.include
@{EnvModel_Parameters.tpl}@********.include
@{EnvModel_Geometry.tpl}@********.include

MODULE EnvModel

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
}@*****.if[@{EM_LESS}@.eval]
)@
@(
   @{EnvModel_Parameters.tpl}@********.include
   @{EnvModel_Scaling.tpl}@********.include
   @{EnvModel_Abstract.tpl}@********.include
)@
}@*********.if[@{FULL_MODEL}@.eval]