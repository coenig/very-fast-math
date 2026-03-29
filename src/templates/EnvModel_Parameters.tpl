@{@{@(-- Found variable #0# with value @{#0#}@*******.eval during generation of EnvModel (default would be #1#).)@
    @(-- Undeclared variable #0# found during generation of EnvModel. Setting to default value #1#. @{@#0# = #1#}@********.eval.nil)@
}@*********.if[@{#0#}@.vfm_variable_declared]}@**********.newMethod[defaultValue, 1]

@{@{@(-- Found variable #0# with value @{#0#}@*******.printHeap during generation of EnvModel (default would be #1#).)@
    @(-- Undeclared variable #0# found during generation of EnvModel. Setting to default value #1#. @{#1#}@********.stringToHeap[#0#])@
}@*********.if[@{#0#}@.vfm_variable_declared]}@**********.newMethod[defaultValueString, 1]

-- Note that the below exact formulation is used in regex for caching. 
-- Do not change...

-- Global options
@{EM_LESS}@*******.defaultValue[false]
@{SCENGEN_MODE}@*******.defaultValue[false]

-- Parametrization of road topology features
@{NUMLANES}@*******.defaultValue[3]                       -- Maximum number of lanes a road SEGMENT can have
@{SEGMENTS}@*******.defaultValue[4]                       -- Number of segments per straight road SECTION with flexible length and NUMLANES
@{SEGMENTSMINLENGTH}@*******.defaultValue[30]         
@{SECTIONS}@*******.defaultValue[3]                       -- Number of straight road SECTIONS which form a road network
@{SECTIONSMAXLENGTH}@*******.defaultValue[150]
@{SECTIONSMINLENGTH}@*******.defaultValue[50]
@{ALLOW_ZEROLENGTH_SECTIONS}@*******.defaultValue[true]
@{ANGLEGRANULARITY}@*******.defaultValue[45]              -- Only every n-th angle (DEG) can be used between two sections at connection points (zero degrees is straight ahead)
@{MAXOUTGOINGCONNECTIONS}@*******.defaultValue[3]         -- Maximum number of successors a straight road may have
@{MINDISTCONNECTIONS}@*******.defaultValue[20]            -- The minimal "X" variable when going from the drain of one road to the source of a connected one
@{MAXDISTCONNECTIONS}@*******.defaultValue[50]            -- The maximal "X" variable when going from the drain of one road to the source of a connected one
@{MAXDISTENDPOINTS}@*******.defaultValue[5]               -- Two end points must be at least this apart (approximated by maxvar method)
@{MODEL_INTERSECTION_GEOMETRY}@*******.defaultValue[true] -- If the length of the junctions is calculated subject to connections etc. (Makes calculation slow, but needed at least for multi-lane road networks.)

-- Parameters for ego and non-ego vehicles (vehicle length is below in LC section)
@{EGOLESS}@*******.defaultValue[false]
@{UCD}@*******.defaultValue[false]
@{NONEGOS}@*******.defaultValue[5]
@{LEFTLC}@*******.defaultValue[true]
@{RIGHTLC}@*******.defaultValue[false]
@{KEEPEGOFIXEDTOLANE}@*******.defaultValue[false]
@{VIPER}@*******.defaultValue[false]
@{LONGCONTROL}@*******.defaultValue[true]
@{DEBUG}@*******.defaultValue[true]
@{MAXSPEEDEGO}@*******.defaultValue[34]
@{MAXSPEEDNONEGO}@*******.defaultValue[70]
@{MINACCELEGO}@*******.defaultValue[-8]
@{MAXACCELEGO}@*******.defaultValue[2]
@{MINACCELNONEGO}@*******.defaultValue[-8]
@{MAXACCELNONEGO}@*******.defaultValue[6]
@{MIN_TIME_BETWEEN_LANECHANGES}@*******.defaultValue[2]
@{SAFETY_DISTANCE_FACTOR_NONEGO}@*******.defaultValue[0.4] -- The factor to multiply the "halber Tacho" safety distance with (one decimal precision).
@{MAXEGOVISRANGE}@*******.defaultValue[250]
@{CLOSEFRONTDIST}@*******.defaultValue[10]
@{CALCULATE_LEFT_GAP}@*******.defaultValue[true]
@{CALCULATE_CENTER_GAP}@*******.defaultValue[true]
@{CALCULATE_RIGHT_GAP}@*******.defaultValue[true]
@{CALCULATE_RIGHT_GAP_REAR}@*******.defaultValue[true] -- Flag to leave out the rear/right gap when not needed, even when right gap is calculated.
@{LONGDISTMAX}@*******.defaultValue[300]         -- This is the max distance to the front and back of ego.
@{INITPOSRANGENONEGOS}@*******.defaultValue[100]  -- Range to front where non-egos can be positioned initially. (TODO: Should be replaced by length of resp. section.)
@{CLOSETOEGOPAR}@*******.defaultValue[80]
@{TIMESCALING}@*******.defaultValue[1000]        -- nondimensionalization constant for time, in milliseconds
@{DISTANCESCALING}@*******.defaultValue[1000]    -- nondimensionalization constant for distance, in millimeters

@{LANES_MAX_SPEEDS}@*******.defaultValue[@(15)@@(16)@@(17)@]

-- Lanechange parameters
@{ANGLE_BASED_LC}@*******.defaultValue[false]          -- Do the angle-based LC as opposed to the "classic" lane-based one.
@{@{LC_ANGLE_GRANULARITY_DEG}@*******.defaultValue[1]  -- Only if ANGLE_BASED_LC -- TODO: Do we need this? For now only full degrees as int.}@*******************.nil
@{MAX_LAT_ACCEL}@*******.defaultValue[1]               -- Only if ANGLE_BASED_LC
@{MIN_LC_DURATION_SEC}@*******.defaultValue[3]         -- Only if ANGLE_BASED_LC
@{MAX_LC_DURATION_SEC}@*******.defaultValue[8]         -- Only if ANGLE_BASED_LC
-- TODO: The above angle-based lanechange is a work in progress and does nothing productive so far.

@{SIMPLE_LC}@*******.defaultValue[false]               -- Only if !ANGLE_BASED_LC: Allow non-egos to switch a "half-lane" if time_since_last_lc has passed (true); otherwise full version by Christian H.

-- Below: Extend the simple LC by allowing more fine-granular steps between the lanes.
-- We do not keep this in synch with the full Christian implementation for now. I.e., use the default parameters to activate this.
@{LATERAL_LC_GRANULARITY}@*******.defaultValue[0]        -- 0 is how it was before, n > 0 increases technical number of lanes by n. Independent of ANGLE_BASED_LC -- Deliberately separating this from scaling (could discuss in future)
@{LANE_WIDTH}@*******.defaultValue[375]                  -- In cm. Default value 400 from highway-env (highway_env/road/lane.py).          Note that we have 375 in vfm...   (include/geometry/images.h)
@{VEHICLE_WIDTH}@*******.defaultValue[185]               -- In cm. Default value 200 from highway-env (highway_env/vehicle/kinematics.py). Note that we have 185.2 in vfm... (include/geometry/images.h)
@{VEHICLE_LENGTH}@*******.defaultValue[5]                -- In m(!). Default value 5.0 from highway-env (highway_env/vehicle/kinematics.py). Note that we have 4.923 in vfm... (include/geometry/images.h)
@{MAX_JUMP_OVER_TECHNICAL_LANES}@*******.defaultValue[1] -- For SIMPLE_LC, how big a step can be in one cycle by changing from one lane to another.


-- Parameters for "skipping" of CEXs.
@{DOUBLEMERGEPROTECTION}@*******.defaultValue[true] -- Prohibits non-ego LCs when in danger of a double-merge
@{BRAKEINHIBITION}@*******.defaultValue[true]       -- Prohibits non-egos in the front gaps to brake so hard that ego could not react on it in one step.
@{HARDBRAKEPREVENTION}@*******.defaultValue[-8]     -- Cuts out traces with very hard decelerations. Set to MINACCELEGO to deactivate.
@{STANDINGCARSUPTOID}@*******.defaultValue[-1]
@{FARMERGINGCARS}@*******.defaultValue[false]
@{ABORTREVOKE}@*******.defaultValue[false]
@{KEEP_EGO_FROM_GREEN}@*******.defaultValue[true]   -- Makes sure, ego never leaves the lanes.

-- Special purpose parameters.
@{FEASIBILITY}@*******.defaultValue[true] -- Includes defines needed for the ACA4.1 Fesibility study.
@{CONCRETE_MODEL}@*******.defaultValue[true] -- Uses the --concrete-- model, as opposed to (deprecated) abstract model be Alberto.
@{MAXSECPARTS}@*******.defaultValue[3] -- TODO: Number of segments a section is split into for the abstract model. Should be replaced by scaling (if possible).

-- EO Do not change.



@{@NUM_TECHNICAL_LANES = NUMLANES + LATERAL_LC_GRANULARITY}@*******.eval.nil
