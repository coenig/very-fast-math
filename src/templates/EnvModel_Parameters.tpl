@{@{@(-- Found variable #0# with value @{#0#}@******.eval during generation of EnvModel (default would be #1#).)@
    @(-- Undeclared variable #0# found during generation of EnvModel. Setting to default value #1#. @{@#0# = #1#}@*******.eval.nil)@
}@********.if[@{#0#}@.vfm_variable_declared]}@*********.newMethod[defaultValue, 1]

-- Note that the below exact formulation is used in regex for caching. 
-- Don't change...

-- Global options
@{EM_LESS}@******.defaultValue[false]

-- Parametrization of road topology features
@{NUMLANES}@******.defaultValue[3]               -- Maximum number of lanes a road SEGMENT can have
@{SEGMENTS}@******.defaultValue[4]               -- Number of segments per straight road SECTION with flexible length and NUMLANES
@{SEGMENTSMINLENGTH}@******.defaultValue[30] 
@{SECTIONS}@******.defaultValue[3]               -- Number of straight road SECTIONS which form a road network
@{SECTIONSMAXLENGTH}@******.defaultValue[250]
@{BORDERLEFT}@******.defaultValue[-1000]         -- Left...
@{BORDERTOP}@******.defaultValue[-1000]          -- Top...
@{BORDERRIGHT}@******.defaultValue[1000]         -- Right...
@{BORDERBOTTOM}@******.defaultValue[1000]        -- Bottom border of the coordinate system the straight roads can be placed on
@{COORDGRANULARITY}@******.defaultValue[100]      -- Only every n-th coordinate in x and y direction can be used to place connection points on
@{ANGLEGRANULARITY}@******.defaultValue[30]      -- Only every n-th angle (DEG) can be used between two sections at connection points (zero degrees is straight ahead)
@{MAXOUTGOINGCONNECTIONS}@******.defaultValue[3] -- Maximum number of successors a straight road may have
@{MINDISTCONNECTIONS}@******.defaultValue[10]    -- Two connection points have to be at least this far apart
@{MAXDISTCONNECTIONS}@******.defaultValue[50]    -- Two connection points may not be more than this apart

-- Parameters for ego and non-ego vehicles
@{EGOLESS}@******.defaultValue[false]
@{NONEGOS}@******.defaultValue[5]
@{LEFTLC}@******.defaultValue[true]
@{RIGHTLC}@******.defaultValue[false]
@{KEEPEGOFIXEDTOLANE}@******.defaultValue[false]
@{VIPER}@******.defaultValue[false]
@{LONGCONTROL}@******.defaultValue[true]
@{DEBUG}@******.defaultValue[true]
@{MAXSPEEDEGO}@******.defaultValue[34]
@{MAXSPEEDNONEGO}@******.defaultValue[70]
@{MINACCELEGO}@******.defaultValue[-8]
@{MAXACCELEGO}@******.defaultValue[2]
@{MINACCELNONEGO}@******.defaultValue[-8]
@{MAXACCELNONEGO}@******.defaultValue[6]
@{MAXEGOVISRANGE}@******.defaultValue[250]
@{CLOSEFRONTDIST}@******.defaultValue[10]
@{LONGDISTMAX}@******.defaultValue[300]         -- This is the max distance to the front and back of ego.
@{INITPOSRANGENONEGOS}@******.defaultValue[200] -- Range to front and back where non-egos can be positioned initially.
@{CLOSETOEGOPAR}@******.defaultValue[80]
@{TIMESCALING}@******.defaultValue[1000]        -- nondimensionalization constant for time, in milliseconds
@{DISTANCESCALING}@******.defaultValue[1000]    -- nondimensionalization constant for distance, in millimeters

-- Parameters for "skipping" of CEXs.
@{DOUBLEMERGEPROTECTION}@******.defaultValue[true] -- Prohibits non-ego LCs when in danger of a double-merge
@{BRAKEINHIBITION}@******.defaultValue[true]       -- Prohibits non-egos in the front gaps to brake so hard that ego could not react on it in one step.
@{HARDBRAKEPREVENTION}@******.defaultValue[-8]     -- Cuts out traces with very hard decelerations. Set to MINACCELEGO to deactivate.
@{STANDINGCARSUPTOID}@******.defaultValue[-1]
@{FARMERGINGCARS}@******.defaultValue[false]
@{ABORTREVOKE}@******.defaultValue[false]
@{KEEP_EGO_FROM_GREEN}@******.defaultValue[true]   -- Makes sure, ego never leaves the lanes.
@{CALCULATE_RIGHT_GAP_REAR}@******.defaultValue[true] -- Flag to leave out the rear/right gap when not needed.

-- Special purpose parameters.
@{FEASIBILITY}@******.defaultValue[true] -- Includes defines needed for the ACA4.1 Fesibility study.


-- EO Don't change.
