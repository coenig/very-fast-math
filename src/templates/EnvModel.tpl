@{EnvModel_Header.tpl}@********.include
@{EnvModel_Timescaling.tpl}@********.include
@{EnvModel_Parameters.tpl}@********.include
@{EnvModel_Geometry.tpl}@********.include

MODULE EnvModel

-- segment l1 and line l2 intersect?
-- @{@{lines(line(vec(l1.begin.x; l1.begin.y); vec(l1.end.x; l1.end.y)); line(vec(l2.begin.x; l2.begin.y); vec(l2.end.x; l2.end.y)))}@.syntacticSegmentAndLineIntersect}@.serializeNuXmv

-- segment l2 and line l1 intersect?
-- @{@{lines(line(vec(l2.begin.x; l2.begin.y); vec(l2.end.x; l2.end.y)); line(vec(l1.begin.x; l1.begin.y); vec(l1.end.x; l1.end.y)))}@.syntacticSegmentAndLineIntersect}@.serializeNuXmv


@{
@( -- EM-less build
   @{scalingList}@*.pushBack[-- no,time]

   @{PLANNER_VARIABLES}@.printHeap
)@
@( -- EM-full build
------------------------------------------
--
-- Begin: Constants and common definitions
--
------------------------------------------

DEFINE
    ---------------- Begin of lc parameterization -----------------

    -- IMPORTANT: The following constraints must be satisfied such that the model works as intended:
    --      1. leave_src_lane_latest >= 1
    --      2. complete_lane_change_latest_after >= 1
    --      3. leave_src_lane_earliest_after <= leave_src_lane_latest_after
    --      4. complete_lane_change_earliest_after  <= complete_lane_change_latest_after
    --      5. leave_src_lane_latest_after <= complete_lane_change_latest_after
    --      5. abort_lane_change_complete_latest_after <= complete_lane_change_latest_after

    @{leave_src_lane_earliest_after}@*.timeWorldToEnvModelDef[1];             -- earliest point in time where the vehicle may cross the lane border, i.e., after this transition the vehicle occupies two lanes 
    @{leave_src_lane_latest_after}@*.timeWorldToEnvModelDef[5];
    @{complete_lane_change_earliest_after}@*.timeWorldToEnvModelDef[1];       -- earliest point in time where the vehicle is entirely on its target lane, i.e., after this transition the vehicle only occupies one lane
    @{complete_lane_change_latest_after}@*.timeWorldToEnvModelDef[7];
    @{abort_lane_change_complete_earliest_after}@*.timeWorldToEnvModelDef[1]; -- earliest point in time where the vehicle is entirely back on its source lane after a lane change abort
    @{abort_lane_change_complete_latest_after}@*.timeWorldToEnvModelDef[3];
    @{min_time_between_lcs}@*.timeWorldToEnvModelDef[1];                      -- after finisihing one lc, how much time needs to pass before the next one may be started

---------------- End of lc parameterization -----------------

    @{a_min}@*.accelerationWorldToEnvModelDef[MINACCELNONEGO];
    @{a_max}@*.accelerationWorldToEnvModelDef[MAXACCELNONEGO];
    @{min_dist_long}@*.distanceWorldToEnvModelDef[-1];                            -- the minimum distance kept by other vehicle to preceding ego, we use -1 meaning one meter behind
    @{veh_length}@*.distanceWorldToEnvModelDef[5];                                -- we assume a vehicle length of 5m for distance calculation to the front
    @{max_vel}@*.velocityWorldToEnvModelDef[MAXSPEEDNONEGO];

    @{ego.max_vel}@*.velocityWorldToEnvModelDef[MAXSPEEDEGO];
    @{ego.min_dist_long}@*.scalingVariable[distance] := veh_length;
    @{ego.following_dist}@*.scalingVariable[distance] := max(@{2}@.timeWorldToEnvModelConst * ego.v, ego.min_dist_long);  -- dynamic following distance according to 2s rule (= THW)
    @{ego.min_accel}@*.accelerationWorldToEnvModelDef[MINACCELEGO];
    @{ego.max_accel}@*.accelerationWorldToEnvModelDef[MAXACCELEGO];

    @{params.turn_signal_duration}@*.timeWorldToEnvModelDef[2]; -- turn signals will be on for this amount of time -- TODO: Connection via vfm-aka not working, has to be investigated.
    -- total lc duration shall be 5 s according to code documentation
    -- src lane is left 1s or 2s after end of turn signal duration, tgt lane is reached after 5s
    @{ego.leave_src_lane_earliest_after}@*.timeWorldToEnvModelDef[1] + params.turn_signal_duration;  -- 1 s after end of turn signal duration
    @{ego.leave_src_lane_latest_after}@*.timeWorldToEnvModelDef[2] + params.turn_signal_duration;    -- 2 s after end of turn signal duration
    @{ego.complete_lane_change_earliest_after}@*.timeWorldToEnvModelDef[2] + params.turn_signal_duration;
    @{ego.complete_lane_change_latest_after}@*.timeWorldToEnvModelDef[2] + params.turn_signal_duration;
    -- it seems that there is no parameter given for the duration of the abort, assume same duration to roll back that it took to get to the current pos
    @{ego.min_time_between_lcs}@*.timeWorldToEnvModelDef[2];                      -- after finisihing one lc, how much time needs to pass before the next one may be started

    @{max_ego_visibility_range}@*.distanceWorldToEnvModelDef[MAXEGOVISRANGE];                      --how far can the ego sensors see? Vehicles being further away than this cannot be detected and will not be considered

    empty_gap_indicator := -1; --Counterpart to i_FREE_LANE in Viper, could be AKA-d but for now should be fine to have twice.
--------------------------------------------------------
-- End: Constants and common definitions
--------------------------------------------------------


--------------------------------------------------------
--
-- Begin: Non-ego Spec (generate once per non-ego Vehicle)
--
--------------------------------------------------------

VAR
   cnt : integer;
   num_lanes : integer;

   @{
   @{segment_[num]_pos_begin}@*.scalingVariable[distance] : integer;
   segment_[num]_min_lane : integer;
   segment_[num]_max_lane : integer;
   }@**.for[[num], 0, @{SEGMENTS - 1}@.eval]

	@{
    -- XVarEnvModelCarNote=@{>>> Car [i] <<<}@***.id
    veh___6[i]9___.do_lane_change : boolean;                   -- signal that a lane change is ongoing
    veh___6[i]9___.abort_lc : boolean;                         -- signal that an ongoing lane change is aborted
    veh___6[i]9___.lc_direction : {ActionDir____LEFT, ActionDir____CENTER, ActionDir____RIGHT};         -- the direction in which the lane change takes place
    veh___6[i]9___.turn_signals : {ActionDir____LEFT, ActionDir____CENTER, ActionDir____RIGHT};         -- the direction in which the turn signals are set
    @{veh___6[i]9___.lc_timer}@*.scalingVariable[time] : -1..complete_lane_change_latest_after;     -- timer such that the lc does eventually finish
    @{veh___6[i]9___.time_since_last_lc}@*.scalingVariable[time] : -1..min_time_between_lcs;        -- enough time has expired since the last lc happened
    veh___6[i]9___.change_lane_now : 0..1;                     -- variable for the non-deterministic choice whether we now change the lane

    @{veh___6[i]9___.rel_pos}@*.scalingVariable[distance] : integer; -- relative position to ego in m, rel_pos < 0 means the rear bumber of the other vehicle is behind the rear bumper of the ego
    @{veh___6[i]9___.prev_rel_pos}@*.scalingVariable[distance] : integer;
    @{veh___6[i]9___.a}@*.scalingVariable[acceleration] : integer;    -- accel in m/s^2, (assume positive accel up to 6m/s^2, which is already a highly tuned sports car)
    @{veh___6[i]9___.v}@*.scalingVariable[velocity] : integer;

    @{
    veh___6[i]9___.lane_b[j] : boolean;
    }@.for[[j], 0, @{NUMLANES - 1}@.eval]

    -- auxialiary variables required for property evaluation
    veh___6[i]9___.lc_leave_src_lane : boolean; -- probably superfluous meanwhile
	
	}@**.for[[i], 0, @{NONEGOS - 1}@.eval]

    tar_dir : {ActionDir____LEFT, ActionDir____CENTER, ActionDir____RIGHT};

   @{
      section___6[sec]9___.source.x : integer;
      section___6[sec]9___.source.y : integer;
      section___6[sec]9___.drain.x : integer;
      section___6[sec]9___.drain.y : integer;
   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]

ASSIGN
   @{
      next(section___6[sec]9___.source.x) := section___6[sec]9___.source.x;
      next(section___6[sec]9___.source.y) := section___6[sec]9___.source.y;
      next(section___6[sec]9___.drain.x) := section___6[sec]9___.drain.x;
      next(section___6[sec]9___.drain.y) := section___6[sec]9___.drain.y;
   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]


   @{
   next(segment_[num]_pos_begin) := segment_[num]_pos_begin;
   next(segment_[num]_min_lane) := segment_[num]_min_lane;
   next(segment_[num]_max_lane) := segment_[num]_max_lane;
   }@.for[[num], 0, @{SEGMENTS - 1}@.eval]

   INIT section___609___.source.x = 0;
   INIT section___609___.source.y = 0;
   -- INIT section___609___.drain.x ==> Not specified, so the length of the section is figured out from the length of the segments.
   INIT section___609___.drain.y = 0;

   @{
      INIT section___6[sec]9___.source.x <= @{BORDERRIGHT}@.eval[0]  & section___6[sec]9___.source.x >= @{BORDERLEFT}@.eval[0];
      INIT section___6[sec]9___.source.y <= @{BORDERBOTTOM}@.eval[0] & section___6[sec]9___.source.y >= @{BORDERTOP}@.eval[0];
      INIT section___6[sec]9___.drain.x  <= @{BORDERRIGHT}@.eval[0]  & section___6[sec]9___.drain.x >= @{BORDERLEFT}@.eval[0];
      INIT section___6[sec]9___.drain.y  <= @{BORDERBOTTOM}@.eval[0] & section___6[sec]9___.drain.y >= @{BORDERTOP}@.eval[0];
   }@**.for[[sec], 1, @{SECTIONS - 1}@.eval]

INVAR tar_dir = ActionDir____LEFT;
INIT cnt = 0;

INIT 0 = @{segment_0_pos_begin}@*.scalingVariable[distance];
INIT segment_@{SEGMENTS - 1}@.eval[0]_pos_begin < @{200}@.distanceWorldToEnvModelConst;
@{
INIT segment_[num]_pos_begin + @{SEGMENTSMINLENGTH}@.distanceWorldToEnvModelConst < segment_@{[num] + 1}@.eval[0]_pos_begin;
INIT abs(segment_[num]_min_lane - segment_@{[num] + 1}@.eval[0]_min_lane) <= 1;
INIT abs(segment_[num]_max_lane - segment_@{[num] + 1}@.eval[0]_max_lane) <= 1;
}@**.for[[num], 0, @{SEGMENTS - 2}@.eval]

@{
INIT segment_[num]_max_lane >= segment_[num]_min_lane;
INIT segment_[num]_min_lane >= 0;
INIT segment_[num]_max_lane <= @{(NUMLANES - 1)}@.eval[0];
}@.for[[num], 0, @{SEGMENTS - 1}@.eval]

INIT segment_0_min_lane = 0 & segment_0_max_lane = @{(NUMLANES - 1)}@.eval[0]; -- Make sure we always have a drivable lane at the start. TODO: Make flexible.

TRANS next(cnt) = cnt + 1;

@{
   @{EnvModel_Feasibility.tpl}@*******.include
}@********.if[@{FEASIBILITY}@.eval]

DEFINE

   large_number := 10000;

   segment_@{SEGMENTS}@.eval[0]_pos_begin := segment_@{SEGMENTS - 1}@.eval[0]_pos_begin + large_number; -- Helper variable to make below loop simpler.

   @{
   @{lane_[lane]_availability_from_segment_@{SEGMENTS}@**.eval[0]}@*.scalingVariable[distance] := 0;          -- Helper variable to make below loop simpler.
   
   @{@{lane_[lane]_availability_from_segment_[seg]}@*.scalingVariable[distance] := case
      segment_[seg]_min_lane <= [lane] & segment_[seg]_max_lane >= [lane] : lane_[lane]_availability_from_segment_@{[seg] + 1}@.eval[0] + segment_@{[seg] + 1}@.eval[0]_pos_begin - segment_[seg]_pos_begin;
      TRUE: 0;
   esac;
   }@**.for[[seg], 0, @{SEGMENTS - 1}@.eval]

   @{ego.lane_[lane]_availability}@*.scalingVariable[distance] := case
      @{ego.abs_pos >= segment_[seg]_pos_begin : lane_[lane]_availability_from_segment_[seg] + segment_[seg]_pos_begin - ego.abs_pos;
      }@.for[[seg], @{SEGMENTS - 1}@.eval, 0, -1]TRUE: 0;
   esac;
   }@***.for[[lane], 0, @{NUMLANES - 1}@.eval]
 
@{
   @{veh___6[i]9___.abs_pos}@*.scalingVariable[distance] := veh___6[i]9___.rel_pos + ego.abs_pos;
}@**.for[[i], 0, @{NONEGOS - 1}@.eval]

@{
-- Make sure ego does not drive on the GREEN.
ego.on_lane_min := case
   @{ego_lane_b[j] : [j];
   }@.for[[j], 0, @{NUMLANES - 2}@.eval]TRUE : @{NUMLANES - 1}@.eval[0];
esac;

ego.on_lane_max := case
   @{ego_lane_b[j] : [j];
   }@.for[[j], @{NUMLANES - 1}@.eval, 1, -1]TRUE : 0;
esac;

INVAR (ego.abs_pos < segment_0_pos_begin) -> 
(ego.on_lane_min >= segment_0_min_lane & ego.on_lane_max <= segment_0_max_lane);

@{
INVAR (ego.abs_pos >= segment_[num]_pos_begin & ego.abs_pos < segment_@{[num] + 1}@.eval[0]_pos_begin) -> 
(ego.on_lane_min >= segment_[num]_min_lane & ego.on_lane_max <= segment_[num]_max_lane);
}@*.for[[num], 0, @{SEGMENTS - 2}@.eval]
INVAR (ego.abs_pos >= segment_@{SEGMENTS - 1}@.eval[0]_pos_begin) -> 
(ego.on_lane_min >= segment_@{SEGMENTS - 1}@.eval[0]_min_lane & ego.on_lane_max <= segment_@{SEGMENTS - 1}@.eval[0]_max_lane);

DEFINE
}@.if[@{KEEP_EGO_FROM_GREEN}@.eval]

-- Make sure non-egos do not drive on the GREEN.
@{
veh___6[i]9___.on_lane_min := case
   @{veh___6[i]9___.lane_b[j] : [j];
   }@.for[[j], 0, @{NUMLANES - 2}@.eval]TRUE : @{NUMLANES - 1}@.eval[0];
esac;

}@*.for[[i], 0, @{NONEGOS - 1}@.eval]
@{

veh___6[i]9___.on_lane_max := case
   @{veh___6[i]9___.lane_b[j] : [j];
   }@.for[[j], @{NUMLANES - 1}@.eval, 1, -1]TRUE : 0;
esac;

}@*.for[[i], 0, @{NONEGOS - 1}@.eval]

@{@{
   -- veh___6[i]9___.prohibit_lanes_up_to_[j] := @{!veh___6[i]9___.lane_b[k]}@.for[[k], 0, [j], 1, &];
}@*.for[[j], 0, @{NUMLANES - 2}@.eval]}@**.for[[i], 0, @{NONEGOS - 1}@.eval].nil

@{@{
   -- veh___6[i]9___.prohibit_lanes_from_[j] := @{!veh___6[i]9___.lane_b[k]}@.for[[k], [j], @{NUMLANES - 1}@.eval, 1, &];
}@*.for[[j], 1, @{NUMLANES - 1}@.eval]}@**.for[[i], 0, @{NONEGOS - 1}@.eval].nil

@{
INVAR (veh___6[i]9___.abs_pos < segment_0_pos_begin) -> 
(veh___6[i]9___.on_lane_min >= segment_0_min_lane & veh___6[i]9___.on_lane_max <= segment_0_max_lane);

@{
INVAR (veh___6[i]9___.abs_pos >= segment_[num]_pos_begin & veh___6[i]9___.abs_pos < segment_@{[num] + 1}@.eval[0]_pos_begin) -> 
(veh___6[i]9___.on_lane_min >= segment_[num]_min_lane & veh___6[i]9___.on_lane_max <= segment_[num]_max_lane);
}@*.for[[num], 0, @{SEGMENTS - 2}@.eval]
INVAR (veh___6[i]9___.abs_pos >= segment_@{SEGMENTS - 1}@.eval[0]_pos_begin) -> 
(veh___6[i]9___.on_lane_min >= segment_@{SEGMENTS - 1}@.eval[0]_min_lane & veh___6[i]9___.on_lane_max <= segment_@{SEGMENTS - 1}@.eval[0]_max_lane);

}@**.for[[i], 0, @{NONEGOS - 1}@.eval]

DEFINE
-- Schematic for ego.right_of_veh_*_lane
--..   ????  ####                     ..--
--..   ????  ####                     ..--
--..   ????  ####         ..          ..--
--..   ????  ####         ..          ..--
--..   ????  ####         ..          ..--
--..   ????  ####         ..          ..--
--..   ????  ####         ..          ..--
--..   ????  ####         ..          ..--
--..          ..          ..          ..--
--..          ..          ..          ..--
--..          ..                      ..--
--..          ..                      ..--
--..   ****  ****  ####   ..          ..--
--..   ****  ****  ####   ..          ..--
--..   ****  ****  ####   ..          ..--
--..   ****  ****  ####   ..          ..--
--..   ****  ****  ####   ..          ..--
--..   ****  ****  ####   ..          ..--
--..   ****  ****  ####   ..          ..--
--..   ****  ****  ####   ..          ..--
--..                                  ..--
--..                                  ..--
--..          ..          ..          ..--
--..          ..          ..          ..--
--..         ****  ****  ####         ..--
--..         ****  ****  ####         ..--
--..         ****  ****  ####         ..--
--..         ****  ****  ####         ..--
--..         ****  ****  ####         ..--
--..         ****  ****  ####         ..--
--..         ****  ****  ####         ..--
--..         ****  ****  ####         ..--
--..          ..          ..          ..--
--..          ..          ..          ..--
--..          ..          ..          ..--
--..          ..          ..          ..--
--..               ****  ****  ####   ..--
--..               ****  ****  ####   ..--
--..          ..   ****  ****  ####   ..--
--..          ..   ****  ****  ####   ..--
--..          ..   ****  ****  ####   ..--
--..          ..   ****  ****  ####   ..--
--..          ..   ****  ****  ####   ..--
--..          ..   ****  ****  ####   ..--


	@{
    -- @{XVarEnvModelCarNote}@***.id
    veh___6[i]9___.lane_0 := veh___6[i]9___.lane_b0 @{& !veh___6[i]9___.lane_b[j]}@.for[[j], 1, @{NUMLANES - 1}@.eval];
    @{
    veh___6[i]9___.lane_@{[k]-1}@.eval[0][k] := @{@{!}@.if[@{[j] != [k] - 1 && [j] != [k]}@.eval]veh___6[i]9___.lane_b[j] }@*.for[[j], 0, @{NUMLANES - 1}@.eval, 1, &];
    veh___6[i]9___.lane_[k] := @{@{!}@.if[@{[k] != [j]}@.eval]veh___6[i]9___.lane_b[j] }@*.for[[j], 0, @{NUMLANES - 1}@.eval, 1, &];
    }@**.for[[k], 1, @{NUMLANES - 1}@.eval]

    veh___6[i]9___.lane_min := veh___6[i]9___.lane_0;
    veh___6[i]9___.lane_max := veh___6[i]9___.lane_@{NUMLANES - 1}@.eval[0];
    veh___6[i]9___.lane_single := @{veh___6[i]9___.lane_[j] }@*.for[[j], 0, @{NUMLANES - 1}@.eval, 1, |];
    veh___6[i]9___.lane_crossing := @{veh___6[i]9___.lane_@{[j]-1}@.eval[0][j]}@*.for[[j], 1, @{NUMLANES - 1}@.eval, 1, |];
    veh___6[i]9___.lane_unchanged := @{veh___6[i]9___.lane_b[j] = next(veh___6[i]9___.lane_b[j])}@.for[[j], 0, @{NUMLANES - 1}@.eval, 1, &];
    veh___6[i]9___.lane_move_down := 
                      (veh___6[i]9___.lane_0 -> next(veh___6[i]9___.lane_0))
                      @{& (veh___6[i]9___.lane_@{[j]-1}@.eval[0][j] -> next(veh___6[i]9___.lane_@{[j]-1}@.eval[0]))
                      & (veh___6[i]9___.lane_[j] -> next(veh___6[i]9___.lane_@{[j]-1}@.eval[0][j]))
                      }@*.for[[j], 1, @{NUMLANES - 1}@.eval];
    veh___6[i]9___.lane_move_up :=
                      (veh___6[i]9___.lane_@{NUMLANES - 1}@.eval[0] -> next(veh___6[i]9___.lane_@{NUMLANES - 1}@.eval[0]))
                      @{& (veh___6[i]9___.lane_@{[j]-1}@.eval[0][j] -> next(veh___6[i]9___.lane_[j]))
                      & (veh___6[i]9___.lane_@{[j]-1}@.eval[0] -> next(veh___6[i]9___.lane_@{[j]-1}@.eval[0][j]))
                      }@*.for[[j], 1, @{NUMLANES - 1}@.eval];

    -- "Soft" counts half a lane as neighboring, too. (Obsolete "regular" mode which counted only exactly one lane distance has been removed.)
    -- TODO: Case missing where ego is between far left/right lane and the one next to it, and the other car is on the resp. extreme lane.
    ego.right_of_veh_[i]_lane := FALSE
       @{| (ego_lane_@{[j]-1}@.eval[0] & veh___6[i]9___.lane_[j]) | (ego_lane_@{[j]-1}@.eval[0] & veh___6[i]9___.lane_~{[j]-1\0}~[j])
       }@*.for[[j], 1, @{NUMLANES - 1}@.eval, 1, | (ego_lane_~{[j]-2\0}~~{[j]-1\0}~ & veh___6[i]9___.lane_~{[j]-1\0}~[j]) | (ego_lane_~{[j]-2\0}~~{[j]-1\0}~ & veh___6[i]9___.lane_~{[j]-1\0}~)];

    ego.left_of_veh_[i]_lane := FALSE
       @{| (ego_lane_[j] & veh___6[i]9___.lane_@{[j]-1}@.eval[0]) | (ego_lane_[j] & veh___6[i]9___.lane_@{[j]-1}@.eval[0][j])
       }@*.for[[j], 1, @{NUMLANES - 1}@.eval, 1, | (ego_lane_~{[j]-1\0}~~{[j]\0}~ & veh___6[i]9___.lane_~{[j]-2\0}~~{[j]-1\0}~) | (ego_lane_~{[j]-1\0}~~{[j]\0}~ & veh___6[i]9___.lane_~{[j]-1\0}~)];

-- ####### TODO: This part is not yet adapted for n lanes (currently only 3 lanes). ####### --
    @{ego.right_right_of_veh_[i]_lane := ego_lane_b1 & (veh___6[i]9___.lane_b1 |veh___6[i]9___.lane_b2 | veh___6[i]9___.lane_b3);
      ego.left_left_of_veh_[i]_lane := ego_lane_b3 & (veh___6[i]9___.lane_b3 |veh___6[i]9___.lane_b2 | veh___6[i]9___.lane_b1);

    ego.close_to_vehicle_[i] := veh___6[i]9___.rel_pos >= -@{CLOSETOEGOPAR}@.distanceWorldToEnvModelConst & veh___6[i]9___.rel_pos < @{CLOSETOEGOPAR}@.distanceWorldToEnvModelConst;

    ego.close_to_vehicle_[i]_on_left_left_lane := ego.right_right_of_veh_[i]_lane & ego.close_to_vehicle_[i];
    ego.close_to_vehicle_[i]_on_right_right_lane := ego.left_left_of_veh_[i]_lane & ego.close_to_vehicle_[i];}@.if[@{FARMERGINGCARS}@.eval]

--    ego.right_of_veh_[i]_lane_gaps := (ego_lane_1 & veh___6[i]9___.lane_b2 & !veh___6[i]9___.lane_b3) |
--	                                (ego_lane_12 & veh___6[i]9___.lane_b2 & !veh___6[i]9___.lane_b1) |
--	                                (ego_lane_2 & veh___6[i]9___.lane_b3) |
--	                                (ego_lane_23 & veh___6[i]9___.lane_3)
--	                               ;
	
--    ego.left_of_veh_[i]_lane_gaps := (ego_lane_3 & veh___6[i]9___.lane_b2 & !veh___6[i]9___.lane_b1) |
--	                                (ego_lane_23 & veh___6[i]9___.lane_b2 & !veh___6[i]9___.lane_b3) |
--	                                (ego_lane_2 & veh___6[i]9___.lane_b1) |
--	                                (ego_lane_12 & veh___6[i]9___.lane_1)
--	                               ;
-- ####### EO TODO: This part is not yet adapted for n lanes (currently only 3 lanes). ####### --

   ego.same_lane_as_veh_[i] := (FALSE
      @{| ((ego_lane_b[j] & veh___6[i]9___.lane_b[j]) @{& !(ego_lane_b@{[j]-1}@.eval[0] & veh___6[i]9___.lane_b@{[j]+1}@.eval[0]) & !(ego_lane_b@{[j]+1}@.eval[0] & veh___6[i]9___.lane_b@{[j]-1}@.eval[0])}@.if[@{[j] > 0 && [j] < NUMLANES - 1}@.eval] )
      }@*.for[[j], 0, @{NUMLANES - 1}@.eval]
   );

	ego.crash_with_veh_[i] := ego.same_lane_as_veh_[i] & (veh___6[i]9___.rel_pos >= -veh_length & veh___6[i]9___.rel_pos <= veh_length);
	ego.blamable_crash_with_veh_[i] := ego.same_lane_as_veh_[i] & (veh___6[i]9___.rel_pos >= 0 & veh___6[i]9___.rel_pos <= veh_length);
	
   @{################# NOTE THAT THIS PART IS CHANGED AS COMPARED TO TACAS VERSION (might be inefficient) ###################}@.nil
   @{################# FORMERLY: veh___6[i]9___.same_lane_as_veh_[j] := ((veh___6[i]9___.lane_b1 & veh___6[j]9___.lane_b1) | (veh___6[i]9___.lane_b2 & veh___6[j]9___.lane_b2) | (veh___6[i]9___.lane_b3 & veh___6[j]9___.lane_b3)); ###################}@.nil
	@{
   veh___6[i]9___.same_lane_as_veh_[k] := (FALSE
      @{| ((veh___6[k]9___.lane_b[j] & veh___6[i]9___.lane_b[j]) @{& !(veh___6[k]9___.lane_b@{[j]-1}@.eval[0] & veh___6[i]9___.lane_b@{[j]+1}@.eval[0]) & !(veh___6[k]9___.lane_b@{[j]+1}@.eval[0] & veh___6[i]9___.lane_b@{[j]-1}@.eval[0])}@.if[@{[j] > 0 && [j] < NUMLANES - 1}@.eval] )
      }@*.for[[j], 0, @{NUMLANES - 1}@.eval]
   );
   }@**.for[[k], 0, @{[i]}@.sub[1]]
	
	}@***.for[[i], 0, @{NONEGOS - 1}@.eval]


    crash := FALSE@{@{}@.space| ego.crash_with_veh_[i]}@*.for[[i], 0, @{NONEGOS - 1}@.eval];
    blamable_crash := FALSE@{@{}@.space| ego.blamable_crash_with_veh_[i]}@*.for[[i], 0, @{NONEGOS - 1}@.eval];
    

DEFINE -- TODO: Formerly INVAR, and "=" instead of ":=" - Check if this is correct!
ego.has_close_vehicle_on_left_left_lane := (FALSE @{@{ | ego.close_to_vehicle_[i]_on_left_left_lane}@.for[[i], 0, @{NONEGOS - 1}@.eval, 1]}@*.if[@{FARMERGINGCARS}@.eval]);     -- For some reason the "FALSE |"...
ego.has_close_vehicle_on_right_right_lane := (FALSE @{@{ | ego.close_to_vehicle_[i]_on_right_right_lane}@.for[[i], 0, @{NONEGOS - 1}@.eval, 1]}@*.if[@{FARMERGINGCARS}@.eval]); -- ...seems to be crucial for runtime.

INVAR
   num_lanes = @{NUMLANES}@.eval[0];

-- Lookup table to speed-up non-linear calculations
DEFINE
square_of_ego_v := case
   @{ego.v = [x] : @{[x] ** 2}@.eval[0];
   }@*.for[[x], 0, @{@{MAXSPEEDEGO}@***.velocityWorldToEnvModelConst - 1}@.eval[0]]
   TRUE: @{@{MAXSPEEDEGO}@***.velocityWorldToEnvModelConst ** 2}@.eval[0];
esac;


@{
-- @{XVarEnvModelCarNote}@**.id
INIT 
   veh___6[i]9___.lane_single;

INIT 
   veh___6[i]9___.rel_pos >= -@{INITPOSRANGENONEGOS}@.distanceWorldToEnvModelConst & veh___6[i]9___.rel_pos <= @{INITPOSRANGENONEGOS}@.distanceWorldToEnvModelConst;

-- Make sure ego and vehicle [i] don't collide in the initial state.
INIT
   veh___6[i]9___.rel_pos < veh_length | veh___6[i]9___.rel_pos > veh_length | !ego.same_lane_as_veh_[i];

INVAR
    veh___6[i]9___.lane_single | veh___6[i]9___.lane_crossing;

INVAR
    (-@{LONGDISTMAX}@.distanceWorldToEnvModelConst <= veh___6[i]9___.rel_pos & veh___6[i]9___.rel_pos <= @{LONGDISTMAX}@.distanceWorldToEnvModelConst) &
    (-@{LONGDISTMAX}@.distanceWorldToEnvModelConst <= veh___6[i]9___.prev_rel_pos & veh___6[i]9___.prev_rel_pos <= @{LONGDISTMAX}@.distanceWorldToEnvModelConst) &
    (max(-veh___6[i]9___.v, a_min) <= veh___6[i]9___.a & veh___6[i]9___.a <= a_max) &
    (0 <= veh___6[i]9___.v & veh___6[i]9___.v <= max_vel);

-- Lookup table to speed-up non-linear calculations
DEFINE
square_of_veh_v_[i] := case
   @{veh___6[i]9___.v = [x] : @{[x] ** 2}@.eval[0];
   }@*.for[[x], 0, @{@{MAXSPEEDNONEGO}@***.velocityWorldToEnvModelConst - 1}@.eval[0]]
   TRUE: @{@{MAXSPEEDNONEGO}@***.velocityWorldToEnvModelConst ** 2}@.eval[0];
esac;

ego_pressured_by_vehicle_[i] := ego.same_lane_as_veh_[i] 
        | (veh___6[i]9___.lc_direction = ActionDir____RIGHT & ego.right_of_veh_[i]_lane & veh___6[i]9___.change_lane_now = 1)
        | (veh___6[i]9___.lc_direction = ActionDir____LEFT & ego.left_of_veh_[i]_lane & veh___6[i]9___.change_lane_now = 1);

ego_pressured_by_vehicle_[i]_from_behind := ego_pressured_by_vehicle_[i] & (veh___6[i]9___.prev_rel_pos < 0);

-- minimum_dist_to_veh_[i] := (veh___6[i]9___.rel_pos + ego_v_minus_veh_[i]_v_square / (2* abs(a_min))); -- This was another way of doing it, though not quite accurate; turns out, it is also slower.

@{minimum_dist_to_veh_[i]}@.scalingVariable[distance] := veh___6[i]9___.rel_pos - square_of_veh_v_[i] / (2*a_min) + square_of_ego_v / (2*a_min);

-- these are invariants that apply to any state
-- check that chosen long acceleration of other vehicle is indeed safe
INVAR
    ego_pressured_by_vehicle_[i]_from_behind ->
            (minimum_dist_to_veh_[i] <= min_dist_long & (veh___6[i]9___.rel_pos < -veh_length));

@{
INVAR -- Non-Ego cars may not collide.
    veh___6[i]9___.same_lane_as_veh_[j] -> (abs(veh___6[j]9___.rel_pos - veh___6[i]9___.rel_pos) > veh_length);

INVAR -- Non-Ego cars may not "jump" over each other.
    !(veh___6[i]9___.same_lane_as_veh_[j] & (veh___6[j]9___.prev_rel_pos < veh___6[i]9___.prev_rel_pos) & (veh___6[j]9___.rel_pos >= veh___6[i]9___.rel_pos)) &
    !(veh___6[i]9___.same_lane_as_veh_[j] & (veh___6[i]9___.prev_rel_pos < veh___6[j]9___.prev_rel_pos) & (veh___6[i]9___.rel_pos >= veh___6[j]9___.rel_pos));
	
}@.for[[j], 0, @{[i]}@.sub[1]]}@**.for[[i], 0, @{NONEGOS - 1}@.eval]



ASSIGN
@{
-- @{XVarEnvModelCarNote}@
    init(veh___6[i]9___.time_since_last_lc) := min_time_between_lcs;       -- init with max value such that lane change is immediately allowed after start
    init(veh___6[i]9___.do_lane_change) := FALSE;
    init(veh___6[i]9___.abort_lc) := FALSE;
    init(veh___6[i]9___.lc_direction) := ActionDir____CENTER;
    init(veh___6[i]9___.lc_timer) := -1;
    init(veh___6[i]9___.change_lane_now) := 0;
    init(veh___6[i]9___.prev_rel_pos) := 0;
    -- init(veh___6[i]9___.v) := @{MAXSPEEDNONEGO / 2}@.velocityWorldToEnvModelConst;
    init(veh___6[i]9___.a) := 0;
    init(veh___6[i]9___.turn_signals) := ActionDir____CENTER;
    init(veh___6[i]9___.lc_leave_src_lane) := FALSE;

    next(veh___6[i]9___.do_lane_change) := 
	    case 
        veh___6[i]9___.do_lane_change = FALSE & veh___6[i]9___.time_since_last_lc >= min_time_between_lcs: {TRUE, FALSE};
        veh___6[i]9___.do_lane_change = TRUE : case 
            -- between earliest and latest point in time, we can finish at any time, but we do not have to - nevertheless, we must occupy both lanes to do so and there cannot be an abort for finishing
            veh___6[i]9___.lc_timer >= complete_lane_change_earliest_after  & veh___6[i]9___.lc_timer < complete_lane_change_latest_after & veh___6[i]9___.lane_crossing & veh___6[i]9___.abort_lc = FALSE: {TRUE, FALSE};  
            veh___6[i]9___.lc_timer = complete_lane_change_latest_after & veh___6[i]9___.lane_crossing & veh___6[i]9___.abort_lc = FALSE: FALSE ;                     -- at the latest point in time, we definitely need to finish
            -- between earliest and latest point in time, we can finish the abort at any time, but we do not have to; only consider timing when occupying both lanes
            veh___6[i]9___.lc_timer >= abort_lane_change_complete_earliest_after & veh___6[i]9___.lc_timer < abort_lane_change_complete_latest_after & veh___6[i]9___.lane_crossing & veh___6[i]9___.abort_lc = TRUE: {TRUE, FALSE};
            veh___6[i]9___.lc_timer = abort_lane_change_complete_latest_after & veh___6[i]9___.lane_crossing & veh___6[i]9___.abort_lc = TRUE: FALSE;    -- at the latest point in time, we definitely need to finish the abort
            -- we are still completely on the source lane and shall abort -> just do so regardless of time constraint
            veh___6[i]9___.lc_timer >= 0 & veh___6[i]9___.abort_lc = TRUE & veh___6[i]9___.lane_single: FALSE; 
            TRUE : veh___6[i]9___.do_lane_change;                                                    -- at all other times, just hold value
        esac;
        TRUE: FALSE;
    esac;

    next(veh___6[i]9___.abort_lc) := case
        veh___6[i]9___.lc_timer > leave_src_lane_latest_after & veh___6[i]9___.lane_single: TRUE; -- we were not able to leave the source lane within the interval (because we were next to ego) -> we must abort
        next(veh___6[i]9___.do_lane_change) = TRUE & veh___6[i]9___.abort_lc = FALSE : {TRUE, FALSE};         -- if a lane change is ongoing, we can non-deterministically abort it
        veh___6[i]9___.abort_lc = TRUE & next(veh___6[i]9___.do_lane_change) = FALSE : FALSE;
        TRUE : veh___6[i]9___.abort_lc;
    esac;

    next(veh___6[i]9___.lc_direction) := case
        -- when we decide to do a lane change, choose direction based on what is allowed and store this value throughout the lc procedure as a reference
        veh___6[i]9___.do_lane_change = FALSE & next(veh___6[i]9___.do_lane_change) = TRUE & veh___6[i]9___.lane_min : ActionDir____LEFT;
        veh___6[i]9___.do_lane_change = FALSE & next(veh___6[i]9___.do_lane_change) = TRUE & veh___6[i]9___.lane_max : ActionDir____RIGHT;
        veh___6[i]9___.do_lane_change = FALSE & next(veh___6[i]9___.do_lane_change) = TRUE : {ActionDir____LEFT, ActionDir____RIGHT};
        -- when the lane change is finished, set back to none
        veh___6[i]9___.do_lane_change = TRUE & next(veh___6[i]9___.do_lane_change) = FALSE : ActionDir____CENTER;
        TRUE: veh___6[i]9___.lc_direction;
    esac;

    next(veh___6[i]9___.lc_leave_src_lane) := case
        veh___6[i]9___.lane_single & next(veh___6[i]9___.lane_crossing): TRUE;
        TRUE: FALSE;
    esac;

    next(veh___6[i]9___.turn_signals) := case
        -- when we decide to do a lane change, set indicators according to chosen lc_direction
        veh___6[i]9___.do_lane_change = FALSE & next(veh___6[i]9___.do_lane_change) = TRUE: next(veh___6[i]9___.lc_direction);
        -- turn off once we have left the source lane (ok? or should we turn off only after reaching the target lane?)
        veh___6[i]9___.do_lane_change = TRUE & veh___6[i]9___.lane_single & next(veh___6[i]9___.lane_crossing) : ActionDir____CENTER;
        veh___6[i]9___.lane_single & next(veh___6[i]9___.abort_lc) = TRUE : ActionDir____CENTER;
        TRUE: veh___6[i]9___.turn_signals;
    esac;

    next(veh___6[i]9___.lc_timer) := case                                                                  -- IMPORTANT: Order of cases is exploitet here!!
        veh___6[i]9___.lc_timer = -1 & veh___6[i]9___.do_lane_change = FALSE & next(veh___6[i]9___.do_lane_change) = TRUE & veh___6[i]9___.abort_lc = FALSE : 0;   -- on the rising edge, set time to 0 and activate
        veh___6[i]9___.lc_timer >= 0 & veh___6[i]9___.lane_single & next(veh___6[i]9___.lane_crossing): 0;                         -- reset counter to 0 when crossing the lane marker to leave source lane
        veh___6[i]9___.lc_timer >= 0 & next(veh___6[i]9___.do_lane_change) = FALSE : -1;                                  -- reset running counter to 0 once the lane change is complete
        veh___6[i]9___.lc_timer >= 0 & veh___6[i]9___.abort_lc = FALSE & next(veh___6[i]9___.abort_lc) = TRUE : 0;                                  -- reset counter to 0 when abort happens
        veh___6[i]9___.lc_timer >= 0 & veh___6[i]9___.lc_timer < complete_lane_change_latest_after : veh___6[i]9___.lc_timer + 1;               -- while we are still changing lanes, increment counter
        TRUE: veh___6[i]9___.lc_timer;                                                                     -- keep counter at its value in all other cases
    esac;

    next(veh___6[i]9___.time_since_last_lc) := case
        veh___6[i]9___.do_lane_change = TRUE & next(veh___6[i]9___.do_lane_change) = FALSE: 0;            -- activate timer when lane change finishes
        veh___6[i]9___.time_since_last_lc >= 0 & veh___6[i]9___.time_since_last_lc < min_time_between_lcs: veh___6[i]9___.time_since_last_lc + 1;    -- increment until threshold is reached, saturate at the threshold (-> else condition)
        veh___6[i]9___.do_lane_change = FALSE & next(veh___6[i]9___.do_lane_change) = TRUE: -1;           -- deactivate timer when new lane change starts
        TRUE: veh___6[i]9___.time_since_last_lc;                                           -- this clause also keeps the var at max value once the max value has been reached
    esac;

    next(veh___6[i]9___.change_lane_now) := case
        veh___6[i]9___.lc_timer >= 0: {0, 1};      -- choose non-deterministically that we either do change or do not change the lane now
        TRUE: 0;
    esac;
	
    -- update position (directly feed-through new velocity)
    next(veh___6[i]9___.prev_rel_pos) := veh___6[i]9___.rel_pos;
    next(veh___6[i]9___.rel_pos) := min(max(veh___6[i]9___.rel_pos + (next(veh___6[i]9___.v) - next(ego.v)), -@{LONGDISTMAX}@.distanceWorldToEnvModelConst), @{LONGDISTMAX}@.distanceWorldToEnvModelConst);

    -- update velocity (directly feed-through newly chosen accel)
    next(veh___6[i]9___.v) := min(max(veh___6[i]9___.v + veh___6[i]9___.a, 0), max_vel);

}@.for[[i], @{STANDINGCARSUPTOID + 1}@.eval, @{NONEGOS - 1}@.eval]

@{
-- @{XVarEnvModelCarNote}@
TRANS
    case
        -- the timer is within the interval where we may leave our source lane, we may transition to any neighbor lane but we do not have to (current lane is also allowed for next state)
        veh___6[i]9___.lc_timer >= leave_src_lane_earliest_after & veh___6[i]9___.lc_timer < leave_src_lane_latest_after & veh___6[i]9___.lane_single & !veh___6[i]9___.abort_lc & !next(veh___6[i]9___.abort_lc): 
        case 
            veh___6[i]9___.lc_direction = ActionDir____LEFT & !veh___6[i]9___.lane_max & !(ego.left_of_veh_[i]_lane & (next(veh___6[i]9___.rel_pos) > min_dist_long & next(veh___6[i]9___.rel_pos) < abs(min_dist_long) + veh_length)) : next(veh___6[i]9___.change_lane_now) = 0 ? veh___6[i]9___.lane_unchanged : veh___6[i]9___.lane_move_up;
            veh___6[i]9___.lc_direction = ActionDir____RIGHT & !veh___6[i]9___.lane_min & !(ego.right_of_veh_[i]_lane & (next(veh___6[i]9___.rel_pos) > min_dist_long & next(veh___6[i]9___.rel_pos) < abs(min_dist_long) + veh_length)) : next(veh___6[i]9___.change_lane_now) = 0 ? veh___6[i]9___.lane_unchanged : veh___6[i]9___.lane_move_down;
            TRUE : veh___6[i]9___.lane_unchanged;
        esac;
        -- at the latest point in time, we need to leave the source lane if we have not already
        veh___6[i]9___.lc_timer = leave_src_lane_latest_after & veh___6[i]9___.lane_single: 
        case
                veh___6[i]9___.lc_direction = ActionDir____LEFT & !veh___6[i]9___.lane_max & !(ego.left_of_veh_[i]_lane & (next(veh___6[i]9___.rel_pos) > min_dist_long & next(veh___6[i]9___.rel_pos) < abs(min_dist_long) + veh_length)): veh___6[i]9___.lane_move_up; 
                veh___6[i]9___.lc_direction = ActionDir____RIGHT & !veh___6[i]9___.lane_min & !(ego.right_of_veh_[i]_lane & (next(veh___6[i]9___.rel_pos) > min_dist_long & next(veh___6[i]9___.rel_pos) < abs(min_dist_long) + veh_length)): veh___6[i]9___.lane_move_down;
                TRUE : veh___6[i]9___.lane_unchanged;
        esac;
        -- lane change is finished in the next step (time conditions are checked at do_lane_change), set state to target lane (we must be on two lanes right now)
        veh___6[i]9___.lc_timer > 0 & next(veh___6[i]9___.do_lane_change) = FALSE & veh___6[i]9___.lane_crossing & !veh___6[i]9___.abort_lc & !next(veh___6[i]9___.abort_lc) : 
        case  
            veh___6[i]9___.lc_direction = ActionDir____LEFT & !veh___6[i]9___.lane_max : veh___6[i]9___.lane_move_up; 
            veh___6[i]9___.lc_direction = ActionDir____RIGHT & !veh___6[i]9___.lane_min : veh___6[i]9___.lane_move_down;
            TRUE : veh___6[i]9___.lane_unchanged;
        esac;
        -- there is an abort running and it is finished in the next step (time conditions are checked at do_lane_change), set state back to source lane (we must be on two lanes right now)
        veh___6[i]9___.lc_timer > 0 & !next(veh___6[i]9___.do_lane_change) & veh___6[i]9___.lane_crossing & veh___6[i]9___.abort_lc: 
        case
            veh___6[i]9___.lc_direction = ActionDir____LEFT & !veh___6[i]9___.lane_min: veh___6[i]9___.lane_move_down ;   
            veh___6[i]9___.lc_direction = ActionDir____RIGHT & !veh___6[i]9___.lane_max: veh___6[i]9___.lane_move_up;
            TRUE : veh___6[i]9___.lane_unchanged;
        esac;
        TRUE: veh___6[i]9___.lane_unchanged;                                      -- hold current value in all other cases
    esac;

}@.for[[i], 0, @{NONEGOS - 1}@.eval]

--------------------------------------------------------
-- End: Non-ego Spec 
--------------------------------------------------------


--------------------------------------------------------
--
-- Begin: Ego Env Model Part (generate once)
--
--------------------------------------------------------

VAR
    ego.flCond_full : boolean; -- conditions for lane change to fast lane (lc allowed and desired)
    ego.slCond_full : boolean; -- conditions for lane change to slow lane (lc allowed and desired)
    ego.abCond_full : boolean; -- conditions for abort of lane change 

    -- Ego physical state
    @{ego.v}@*.scalingVariable[velocity] : 0..ego.max_vel;
    @{ego.abs_pos}@*.scalingVariable[distance] : integer;

    @{
    ego_lane_b[j] : boolean;
    }@.for[[j], 0, @{NUMLANES - 1}@.eval]

    env_state.time_scaling_factor : 0..1;  -- TODO: Dummy variable, needs to go away.
    
    ego.on_lane : integer;                        -- TODO: Dummy variable, still used in specs and/or evaluation.cpp. Needs to go away...
    env_state.max_lane_id : integer;              -- TODO: Dummy variable, still used in specs and/or evaluation.cpp. Needs to go away...
    @{env_state.v_max_ego}@*.scalingVariable[velocity] : integer;                -- TODO: Dummy variable, still used in specs and/or evaluation.cpp. Needs to go away...
    @{env_state.max_ego_visibility_range}@*.scalingVariable[distance] : integer; -- TODO: Dummy variable, still used in specs and/or evaluation.cpp. Needs to go away...
    ego.slow_lane_deadlock_active : boolean;      -- TODO: Dummy variable, still used in specs and/or evaluation.cpp. Needs to go away... (Remove from init(...) as well)
    ego.action.type : {ActionType____UNDEFINED, ActionType____OFFROAD, ActionType____LANE_FOLLOWING, ActionType____LANECHANGE_LEFT, ActionType____LANECHANGE_RIGHT, ActionType____LANECHANGE_ABORT, ActionType____CONVERGE_TO_LANE, ActionType____EGO_OPEN_LOOP, ActionType____IN_COLLISION};      -- TODO: Dummy variable, still used in specs and/or evaluation.cpp. Needs to go away...
    ego.action.t_action : integer;                -- TODO: Dummy variable, still used in specs and/or evaluation.cpp. Needs to go away...
    
    ego.lc_direction : {ActionDir____LEFT, ActionDir____RIGHT, ActionDir____CENTER};
    ego.mode : {ActionType____LANE_FOLLOWING@{, ActionType____LANECHANGE_RIGHT}@.if[@{RIGHTLC}@.eval]@{, ActionType____LANECHANGE_LEFT}@.if[@{LEFTLC}@.eval], ActionType____LANECHANGE_ABORT};
    ego.change_lane_now : 0..1;                     -- variable for the non-deterministic choice whether we now change the lane
    @{ego_timer}@*.scalingVariable[time] : -1..@{10}@.timeWorldToEnvModelConst;
    @{ego.time_since_last_lc}@*.scalingVariable[time] : -1..ego.min_time_between_lcs;        -- enough time has expired since the last lc happened

    -- Ego env_model interface
    @{ego.gaps___609___.s_dist_front}@*.scalingVariable[distance] : integer;
    @{ego.gaps___609___.s_dist_rear}@*.scalingVariable[distance] : integer;
    ego.gaps___609___.i_agent_front : empty_gap_indicator..@{NONEGOS-1}@.eval[0];
    ego.gaps___609___.i_agent_rear : empty_gap_indicator..@{NONEGOS-1}@.eval[0];

    @{ego.gaps___619___.s_dist_front}@*.scalingVariable[distance] : integer;
    @{ego.gaps___619___.s_dist_rear}@*.scalingVariable[distance] : integer;
    ego.gaps___619___.i_agent_front : empty_gap_indicator..@{NONEGOS-1}@.eval[0];
    ego.gaps___619___.i_agent_rear : empty_gap_indicator..@{NONEGOS-1}@.eval[0];

    @{ego.gaps___629___.s_dist_front}@*.scalingVariable[distance] : integer;
    @{ego.gaps___629___.s_dist_rear}@*.scalingVariable[distance] : integer;
    ego.gaps___629___.i_agent_front : empty_gap_indicator..@{NONEGOS-1}@.eval[0];
    ego.gaps___629___.i_agent_rear : -1..-1;

@{
    #ERROR: ASSIGN structure for gaps not supported anymore since 2024-06-20.
}@.if[@{NONEGOS <= THRESHOLD_FOR_USING_ASSIGNS_IN_GAP_STRUCTURE}@.eval]

DEFINE
    ego_lane_0 := ego_lane_b0 @{& !ego_lane_b[j]}@.for[[j], 1, @{NUMLANES - 1}@.eval];
    @{
    ego_lane_@{[k]-1}@.eval[0][k] := @{@{!}@.if[@{[j] != [k] - 1 && [j] != [k]}@.eval]ego_lane_b[j] }@*.for[[j], 0, @{NUMLANES - 1}@.eval, 1, &];
    ego_lane_[k] := @{@{!}@.if[@{[k] != [j]}@.eval]ego_lane_b[j] }@*.for[[j], 0, @{NUMLANES - 1}@.eval, 1, &];
    }@**.for[[k], 1, @{NUMLANES - 1}@.eval]

    ego_lane_min := ego_lane_0;
    ego_lane_max := ego_lane_@{NUMLANES - 1}@.eval[0];
    ego_lane_max_free := !ego_lane_b@{NUMLANES - 1}@.eval[0];

    ego_lane_single := @{ego_lane_[j] }@*.for[[j], 0, @{NUMLANES - 1}@.eval, 1, |];
    ego_lane_crossing := @{ego_lane_@{[j]-1}@.eval[0][j]}@*.for[[j], 1, @{NUMLANES - 1}@.eval, 1, |];
    ego_lane_unchanged := @{ego_lane_b[j] = next(ego_lane_b[j])}@.for[[j], 0, @{NUMLANES - 1}@.eval, 1, &];

    ego_lane_move_down := 
                      (ego_lane_0 -> next(ego_lane_0))
                      @{& (ego_lane_@{[j]-1}@.eval[0][j] -> next(ego_lane_@{[j]-1}@.eval[0]))
                      & (ego_lane_[j] -> next(ego_lane_@{[j]-1}@.eval[0][j]))
                      }@*.for[[j], 1, @{NUMLANES - 1}@.eval];
    ego_lane_move_up :=
                      (ego_lane_@{NUMLANES - 1}@.eval[0] -> next(ego_lane_@{NUMLANES - 1}@.eval[0]))
                      @{& (ego_lane_@{[j]-1}@.eval[0][j] -> next(ego_lane_[j]))
                      & (ego_lane_@{[j]-1}@.eval[0] -> next(ego_lane_@{[j]-1}@.eval[0][j]))
                      }@*.for[[j], 1, @{NUMLANES - 1}@.eval];

INVAR
    ego_lane_single | ego_lane_crossing;

INIT ego_lane_single;

INIT ego.abs_pos = 0;

-- TODO: Needed?
@{
INVAR
    (!(ego.abCond_full & ego.mode = ActionType____LANE_FOLLOWING));  -- only abort when a lane change is ongoing
}@.if[@{ABORTREVOKE}@.eval]

INVAR
    (0 <= ego.v & ego.v <= ego.max_vel) &
    (@{max(HARDBRAKEPREVENTION, MINACCELEGO)}@.accelerationWorldToEnvModelConst <= ego.a & ego.a <= ego.max_accel);

INVAR
    (ego.gaps___609___.s_dist_front >= 0 & ego.gaps___609___.s_dist_front <= max_ego_visibility_range + 1) &
    (@{ego.gaps___609___.v_front}@*.scalingVariable[velocity] >=  0 & ego.gaps___609___.v_front <= max_vel) &
    (ego.gaps___609___.s_dist_rear >= 0 & ego.gaps___609___.s_dist_rear <= max_ego_visibility_range + 1) &
    (@{ego.gaps___609___.v_rear}@*.scalingVariable[velocity] >= 0 & ego.gaps___609___.v_rear <= max_vel) &
    (ego.gaps___619___.s_dist_front >= 0 & ego.gaps___619___.s_dist_front <= max_ego_visibility_range + 1) &
    (@{ego.gaps___619___.v_front}@*.scalingVariable[velocity] >= 0 & ego.gaps___619___.v_front <= max_vel) & 
    (ego.gaps___629___.s_dist_front >= 0 & ego.gaps___629___.s_dist_front <= max_ego_visibility_range + 1) &
    (@{ego.gaps___629___.v_front}@*.scalingVariable[velocity] >= 0 & ego.gaps___629___.v_front <= max_vel);

ASSIGN
    init(env_state.time_scaling_factor) := 1; -- TODO: Dummy variable, needs to go away.
    next(env_state.time_scaling_factor) := 1; -- TODO: Dummy variable, needs to go away.
    
    init(ego.slow_lane_deadlock_active) := FALSE;
    next(ego.slow_lane_deadlock_active) := FALSE; -- TODO: For now we don't model slow-lane deadlock

@{ -- Using ASSIGN structure for gaps due to @{NONEGOS}@.eval[0] <= @{THRESHOLD_FOR_USING_ASSIGNS_IN_GAP_STRUCTURE}@.eval[0] non-ego agents.
    init(ego.gaps___609___.v_front) := max_vel; -- Max velocity is indicator of empty gap to the front, 0, to the rear.
    init(ego.gaps___609___.turn_signals_front) := ActionDir____CENTER; 
    init(ego.gaps___609___.v_rear) := 0;   -- Max velocity is indicator of empty gap to the front, 0, to the rear.
    init(ego.gaps___619___.v_front) := max_vel; -- Max velocity is indicator of empty gap to the front, 0, to the rear.
    init(ego.gaps___629___.v_front) := max_vel; -- Max velocity is indicator of empty gap to the front, 0, to the rear.
    init(ego.gaps___629___.turn_signals_front) := ActionDir____CENTER; 
}@.if[@{NONEGOS <= THRESHOLD_FOR_USING_ASSIGNS_IN_GAP_STRUCTURE}@.eval]

    init(ego.lc_direction) := ActionDir____CENTER;
    init(ego.mode) := ActionType____LANE_FOLLOWING;
    init(ego_timer) := -1;
    init(ego.change_lane_now) := 0;
  
    init(ego.time_since_last_lc) := ego.min_time_between_lcs;       -- init with max value such that lane change is immediately allowed after start

    -- ego.v 
    next(ego.v) := min(max(ego.v + ego.a, 0), ego.max_vel);
    next(ego.abs_pos) := ego.abs_pos + ego.v;


-- check that non-ego drives non-maliciously, i.e., behavior does not lead to inevitable crash with ego
DEFINE
ego_pressured_from_ahead_on_own_lane   := ego.gaps___619___.i_agent_front != empty_gap_indicator;
ego_pressured_from_ahead_on_left_lane  := ego.gaps___609___.i_agent_front != empty_gap_indicator & (ego.gaps___609___.turn_signals_front = ActionDir____RIGHT @{| ego.mode = ActionType____LANECHANGE_LEFT}@.if[@{LEFTLC}@.eval]);
ego_pressured_from_ahead_on_right_lane := ego.gaps___629___.i_agent_front != empty_gap_indicator & (ego.gaps___629___.turn_signals_front = ActionDir____LEFT @{| ego.mode = ActionType____LANECHANGE_RIGHT}@.if[@{RIGHTLC}@.eval]);

@{
-- Prohibit the cars in ego's front gaps from braking harder than ego could possibly react on.
INVAR
    ego_pressured_from_ahead_on_own_lane   -> (ego.gaps___619___.a_front > veh_length - ego.gaps___619___.s_dist_front - ego.gaps___619___.v_front + ego.v + a_min);

INVAR
    ego_pressured_from_ahead_on_left_lane  -> (ego.gaps___609___.a_front > veh_length - ego.gaps___609___.s_dist_front - ego.gaps___609___.v_front + ego.v + a_min);

INVAR
    ego_pressured_from_ahead_on_right_lane -> (ego.gaps___629___.a_front > veh_length - ego.gaps___629___.s_dist_front - ego.gaps___629___.v_front + ego.v + a_min);
}@.if[@{BRAKEINHIBITION}@.eval]

DEFINE
   @{allowed_ego_a_front_center}@*.scalingVariable[acceleration] := max(min( ego.gaps___619___.s_dist_front + ego.gaps___619___.v_front + ego.gaps___619___.a_front - ego.v - ego.following_dist, ego.max_accel), ego.min_accel);
   @{allowed_ego_a_front_right}@*.scalingVariable[acceleration] := max(min( ego.gaps___629___.s_dist_front + ego.gaps___629___.v_front + ego.gaps___629___.a_front - ego.v - ego.following_dist, ego.max_accel), ego.min_accel);
   @{allowed_ego_a_front_left}@*.scalingVariable[acceleration] := max(min( ego.gaps___609___.s_dist_front + ego.gaps___609___.v_front + ego.gaps___609___.a_front - ego.v - ego.following_dist, ego.max_accel), ego.min_accel);

@{@(
   -- we need to control speed if a) there is a vehicle on our lane in front (gap[1]) or b) there is a vehicle on the left lane indicating a lane change to our lane (gap[0])
   -- or c) there is a vehicle on the right lane indicating a lane change to our lane (gap[2])
   -- for distances less than 10 m, we go for max brake (or to 0), in all other cases, we take the minimum of the accel allowed by the three cases listed above
   -- Per default not included, the planner is supposed to implement longitudinal control. Set LONGCONTROL = true to activate this.
   ego.a := case
          (ego.gaps___619___.s_dist_front < @{CLOSEFRONTDIST}@.distanceWorldToEnvModelConst 
        | (ego.gaps___609___.s_dist_front < @{CLOSEFRONTDIST}@.distanceWorldToEnvModelConst & ego_pressured_from_ahead_on_left_lane) 
        | (ego.gaps___629___.s_dist_front < @{CLOSEFRONTDIST}@.distanceWorldToEnvModelConst & ego_pressured_from_ahead_on_right_lane)): max(-1*ego.v, ego.min_accel);
        TRUE: min(                                           allowed_ego_a_front_center, 
                min(ego_pressured_from_ahead_on_right_lane ? allowed_ego_a_front_right : ego.max_accel, 
                    ego_pressured_from_ahead_on_left_lane ? allowed_ego_a_front_left  : ego.max_accel));
   esac;
   )@
   @(
VAR
   @{ego.a}@*.scalingVariable[acceleration] : integer;
   )@
}@.if[@{LONGCONTROL}@.eval]

@{
-- Prohibit continuing LC in case of abort condition already active.
TRANS (next(ego.mode = ActionType____LANECHANGE_ABORT) | ego.mode = ActionType____LANECHANGE_ABORT | next(ego.abCond_full) | ego.abCond_full) -> (!ego_lane_move_up & !ego_lane_move_down);
}@.if[@{ABORTREVOKE}@.eval]

ASSIGN
    @{possible_lc_modes=@{
    @{@(@(LEFT)@)@}@*.if[@{LEFTLC}@.eval]
    @{@(@(RIGHT)@)@}@*.if[@{RIGHTLC}@.eval]
    }@}@.nil

    next(ego.lc_direction) := case
        @{ego.mode = ActionType____LANE_FOLLOWING & ego.flCond_full: ActionDir____LEFT;}@.if[@{LEFTLC}@.eval]
        @{ego.mode = ActionType____LANE_FOLLOWING & ego.slCond_full: ActionDir____RIGHT;}@.if[@{RIGHTLC}@.eval]
        next(ego.mode) = ActionType____LANE_FOLLOWING: ActionDir____CENTER;
        TRUE: ego.lc_direction;
    esac;

    next(ego.mode) := case
    @{
        (FALSE @{| ego.mode = ActionType____LANECHANGE_[DIR]}@.for[[DIR], possible_lc_modes]) & (ego.abCond_full | next(ego.abCond_full)): ActionType____LANECHANGE_ABORT;
    }@.if[@{!ABORTREVOKE}@.eval]
        @{ego.mode = ActionType____LANE_FOLLOWING & ego.flCond_full: ActionType____LANECHANGE_LEFT;}@.if[@{LEFTLC}@.eval]
        @{ego.mode = ActionType____LANE_FOLLOWING & ego.slCond_full: ActionType____LANECHANGE_RIGHT;}@.if[@{RIGHTLC}@.eval]
        @{ego.mode = ActionType____LANECHANGE_LEFT & ego_timer = ego.complete_lane_change_latest_after: ActionType____LANE_FOLLOWING;}@.if[@{LEFTLC}@.eval]
        @{ego.mode = ActionType____LANECHANGE_RIGHT & ego_timer = ego.complete_lane_change_latest_after: ActionType____LANE_FOLLOWING;}@.if[@{RIGHTLC}@.eval]
        @{ego.mode = ActionType____LANECHANGE_LEFT & ego_timer >= ego.complete_lane_change_earliest_after  & ego_timer < ego.complete_lane_change_latest_after : {ActionType____LANE_FOLLOWING, ActionType____LANECHANGE_LEFT};}@.if[@{LEFTLC}@.eval]
        @{ego.mode = ActionType____LANECHANGE_RIGHT & ego_timer >= ego.complete_lane_change_earliest_after  & ego_timer < ego.complete_lane_change_latest_after : {ActionType____LANE_FOLLOWING, ActionType____LANECHANGE_RIGHT};}@.if[@{RIGHTLC}@.eval]
    @{
        (FALSE @{| ego.mode = ActionType____LANECHANGE_[DIR]}@.for[[DIR], possible_lc_modes]) & ego.abCond_full: ActionType____LANECHANGE_ABORT;
    }@.if[@{ABORTREVOKE}@.eval]
        ego.mode = ActionType____LANECHANGE_ABORT & ego_timer = 0: ActionType____LANE_FOLLOWING;
        TRUE: ego.mode;
    esac;

    next(ego_timer) := case                                                                 -- IMPORTANT: Order of cases is exploitet here!!
        ego_timer = -1 & ego.mode = ActionType____LANE_FOLLOWING & (FALSE @{| next(ego.mode) = ActionType____LANECHANGE_[DIR]}@.for[[DIR], possible_lc_modes]) : 0;         -- on the rising edge, set time to 0 and activate
        ego_timer >= 0 & next(ego.mode) = ActionType____LANE_FOLLOWING: -1;                                  -- deactivate counter once lane change is complete 
        ego_timer >= 0 & (FALSE @{| ego.mode = ActionType____LANECHANGE_[DIR]}@.for[[DIR], possible_lc_modes]) & next(ego.mode) = ActionType____LANECHANGE_ABORT & ego_timer >= params.turn_signal_duration: ego_timer - params.turn_signal_duration; -- going back takes the same time as going forward, but do not use turn signal duration for abort
        ego_timer > 0 & ego.mode = ActionType____LANECHANGE_ABORT: ego_timer - 1;                        -- count down back to zero while aborting
        ego_timer >= 0 & ego_timer < ego.complete_lane_change_latest_after : ego_timer + 1;   -- while we are still changing lanes, increment counter
        TRUE: ego_timer;                                                                     -- keep counter at its value in all other cases
    esac;

    next(ego.time_since_last_lc) := case
        (@{ego.mode = ActionType____LANECHANGE_[DIR] | }@.for[[DIR], possible_lc_modes] ego.mode = ActionType____LANECHANGE_ABORT) & next(ego.mode) = ActionType____LANE_FOLLOWING : 0;            -- activate timer when lane change finishes
        ego.time_since_last_lc >= 0 & ego.time_since_last_lc < min_time_between_lcs: ego.time_since_last_lc + 1;    -- increment until threshold is reached, saturate at the threshold (-> else condition)
        ego.mode = ActionType____LANE_FOLLOWING & (FALSE @{| next(ego.mode) = ActionType____LANECHANGE_[DIR]}@.for[[DIR], possible_lc_modes]): -1;           -- deactivate timer when new lane change starts
        TRUE: ego.time_since_last_lc;                                           -- this clause also keeps the var at max value once the max value has been reached
    esac;

    next(ego.change_lane_now) := case
        ego_timer >= 0: {0, 1};      -- choose non-deterministically that we either do change or do not change the lane now
        TRUE: 0;
    esac;
    

TRANS
    --next(ego.on_lane) :=
    case
        -- the timer is within the interval where we may leave our source lane, we may transition to any neighbor lane but we do not have to (current lane is also allowed for next state)
        ego_timer >= ego.leave_src_lane_earliest_after & ego_timer < ego.leave_src_lane_latest_after & ego_lane_single & ego.mode != ActionType____LANECHANGE_ABORT : case 
            ego.lc_direction = ActionDir____LEFT & !ego_lane_max: next(ego.change_lane_now) = 0 ? ego_lane_unchanged : ego_lane_move_up; 
            ego.lc_direction = ActionDir____RIGHT & !ego_lane_min: next(ego.change_lane_now) = 0 ? ego_lane_unchanged : ego_lane_move_down;
            TRUE : ego_lane_unchanged;
        esac;
        -- at the latest point in time, we need to leave the source lane if we have not already
        ego_timer = ego.leave_src_lane_latest_after & ego_lane_single & ego.mode != ActionType____LANECHANGE_ABORT: case
            ego.lc_direction = ActionDir____LEFT & !ego_lane_max: ego_lane_move_up; 
            ego.lc_direction = ActionDir____RIGHT & !ego_lane_min: ego_lane_move_down;
            TRUE : ego_lane_unchanged;
        esac;
        -- lane change is finished in the next step (time conditions are checked at do_lane_change), set state to target lane (we must be on two lanes right now)
        ego_timer > 0 & next(ego.mode) = ActionType____LANE_FOLLOWING & ego_lane_crossing & ego.mode != ActionType____LANECHANGE_ABORT: case  
            ego.lc_direction = ActionDir____LEFT & !ego_lane_max: ego_lane_move_up; 
            ego.lc_direction = ActionDir____RIGHT & !ego_lane_min: ego_lane_move_down;
            TRUE : ego_lane_unchanged;
        esac;
        -- there is an abort running and it is finished in the next step (time conditions are checked at do_lane_change), set state back to source lane (we must be on two lanes right now)
        ego_timer = 0 & next(ego.mode) = ActionType____LANE_FOLLOWING & ego_lane_crossing & ego.mode = ActionType____LANECHANGE_ABORT: case
            ego.lc_direction = ActionDir____LEFT & !ego_lane_min: ego_lane_move_down ;   
            ego.lc_direction = ActionDir____RIGHT & !ego_lane_max: ego_lane_move_up;
            TRUE : ego_lane_unchanged;
        esac;
        TRUE: ego_lane_unchanged;                                      -- hold current value in all other cases
    esac;

--------------------------------------------------------
-- End: Ego Env Model Part 
--------------------------------------------------------

--------------------------------------------------------
--
-- Begin: Gap Structure --> Structure changes based on num of non-ego vehicles
--
--------------------------------------------------------

-- Gap calculation has been done in two ways, (1) using assigns and (2) using defines and invars.
-- (2) is more efficient at least for >= 3 non-ego cars, but probably for >= 2 non-ego cars,
-- and of negligible difference for 1 non-ego car, therefore, we currently generate always the (2) version.
-- Nonetheless, the switch remains in place, only the threshold is to be set to 0.

DEFINE
@{
-- @{XVarEnvModelCarNote}@
    veh___6[i]9___.is_visible := abs(veh___6[i]9___.rel_pos) <= max_ego_visibility_range;
    veh___6[i]9___.is_in_front := veh___6[i]9___.rel_pos >= 0;
    veh___6[i]9___.is_behind := veh___6[i]9___.rel_pos < 0;
    veh___6[i]9___.is_visible_in_front := veh___6[i]9___.is_in_front & veh___6[i]9___.is_visible;
    veh___6[i]9___.is_visible_behind := veh___6[i]9___.is_behind & veh___6[i]9___.is_visible;
	
}@.for[[i], 0, @{NONEGOS - 1}@.eval]

@{
GENERATE FOR GAP0:
          XVarGap0vfront=@{true}@***.id
      XVarGap0sdistfront=@{true}@***.id
XVarGap0turnsignalsfront=@{true}@***.id
           XVarGap0vrear=@{true}@***.id
       XVarGap0sdistrear=@{true}@***.id
          XVarGap0afront=@{true}@***.id

GENERATE FOR GAP1:
          XVarGap1vfront=@{true}@***.id
      XVarGap1sdistfront=@{true}@***.id
XVarGap1turnsignalsfront=@{false}@***.id
           XVarGap1vrear=@{true}@***.id
       XVarGap1sdistrear=@{false}@***.id
          XVarGap1afront=@{true}@***.id

GENERATE FOR GAP2:
          XVarGap2vfront=@{true}@***.id
      XVarGap2sdistfront=@{true}@***.id
XVarGap2turnsignalsfront=@{true}@***.id
           XVarGap2vrear=@{true}@***.id
       XVarGap2sdistrear=@{false}@***.id
          XVarGap2afront=@{true}@***.id
}@.nil

@{
@( 
    #ERROR: ASSIGN structure for gaps not supported anymore since 2024-06-20.
)@
@( -- Using INVAR structure for gaps due to @{NONEGOS}@.eval[0] > @{THRESHOLD_FOR_USING_ASSIGNS_IN_GAP_STRUCTURE}@.eval[0] cars.

-- Explanation of the 6 TRANS statements per gap for s_dist and i_agent:
-- s_dist_front is one of the rel_pos-s of the cars in front of us in the correct lane.
-- s_dist_front is the one rel_pos closest to us, if any; if not, the only remaining possible value is max_ego_visibility_range + 1.
-- i_agent_front is the agent id of the car with the resp. s_dist_front, if it is on the correct lane. Should there be no such car, the only remaining value is empty_gap_indicator.
-- s_dist_rear is one of the rel_pos-s of the cars behind us in the correct lane.
-- s_dist_rear is the one negated rel_pos closest to us, if any; if not, the only remaining possible value is max_ego_visibility_range + 1.
-- i_agent_rear is the agent id of the car with the resp. s_dist_rear, if it is on the correct lane. Should there be no such car, the only remaining value is empty_gap_indicator.

-- GAPS[0] ==> Left of Ego
-- FRONT
INVAR
@{(ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front & (ego.gaps___609___.s_dist_front = veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
(ego.gaps___609___.s_dist_front = max_ego_visibility_range + 1);

INVAR
@{((ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front) -> (ego.gaps___609___.s_dist_front <= veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
((@{!(ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front) &
}@.for[[i], 0, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___609___.s_dist_front = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___609___.s_dist_front = veh___6[i]9___.rel_pos - veh_length) & ego.right_of_veh_[i]_lane) <-> ego.gaps___609___.i_agent_front = [i]) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
TRUE;

-- REAR
INVAR
@{(ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind & (ego.gaps___609___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
(ego.gaps___609___.s_dist_rear = max_ego_visibility_range + 1);

INVAR
@{((ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind) -> (ego.gaps___609___.s_dist_rear <= -veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
((@{!(ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind) &
}@.for[[i], 0, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___609___.s_dist_rear = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___609___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length) & ego.right_of_veh_[i]_lane) <-> ego.gaps___609___.i_agent_rear = [i]) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
TRUE;


-- GAPS[1] ==> On Ego lane
-- FRONT
INVAR
@{(ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_in_front & (ego.gaps___619___.s_dist_front = veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
(ego.gaps___619___.s_dist_front = max_ego_visibility_range + 1);

INVAR
@{((ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_in_front) -> (ego.gaps___619___.s_dist_front <= veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
((@{!(ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_in_front) &
}@.for[[i], 0, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___619___.s_dist_front = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___619___.s_dist_front = veh___6[i]9___.rel_pos - veh_length) & ego.same_lane_as_veh_[i]) <-> ego.gaps___619___.i_agent_front = [i]) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
TRUE;

--REAR
INVAR
@{(ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_behind & (ego.gaps___619___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
(ego.gaps___619___.s_dist_rear = max_ego_visibility_range + 1);

INVAR
@{((ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_behind) -> (ego.gaps___619___.s_dist_rear <= -veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
((@{!(ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_behind) &
}@.for[[i], 0, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___619___.s_dist_rear = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___619___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length) & ego.same_lane_as_veh_[i]) <-> ego.gaps___619___.i_agent_rear = [i]) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
TRUE;


-- GAPS[2] ==> Right of Ego
-- FRONT
INVAR
@{(ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front & (ego.gaps___629___.s_dist_front = veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
(ego.gaps___629___.s_dist_front = max_ego_visibility_range + 1);

INVAR
@{((ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front) -> (ego.gaps___629___.s_dist_front <= veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
((@{!(ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front) &
}@.for[[i], 0, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___629___.s_dist_front = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___629___.s_dist_front = veh___6[i]9___.rel_pos - veh_length) & ego.left_of_veh_[i]_lane) <-> ego.gaps___629___.i_agent_front = [i]) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
TRUE;

--REAR
@{
INVAR
@{(ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind & (ego.gaps___629___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
(ego.gaps___629___.s_dist_rear = max_ego_visibility_range + 1);

INVAR
@{((ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind) -> (ego.gaps___629___.s_dist_rear <= -veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
((@{!(ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind) &
}@.for[[i], 0, @{NONEGOS - 1}@.eval] TRUE) -> (ego.gaps___629___.s_dist_rear = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___629___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length) & ego.left_of_veh_[i]_lane) <-> ego.gaps___629___.i_agent_rear = [i]) & 
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
TRUE;
}@.if[@{CALCULATE_RIGHT_GAP_REAR}@.eval]

INVAR -1 <= ego.gaps___609___.i_agent_front & ego.gaps___609___.i_agent_front <= @{NONEGOS - 1}@.eval[0];
INVAR -1 <= ego.gaps___619___.i_agent_front & ego.gaps___619___.i_agent_front <= @{NONEGOS - 1}@.eval[0];
INVAR -1 <= ego.gaps___629___.i_agent_front & ego.gaps___629___.i_agent_front <= @{NONEGOS - 1}@.eval[0];
INVAR -1 <= ego.gaps___609___.i_agent_rear & ego.gaps___609___.i_agent_rear <= @{NONEGOS - 1}@.eval[0];
INVAR -1 <= ego.gaps___619___.i_agent_rear & ego.gaps___619___.i_agent_rear <= @{NONEGOS - 1}@.eval[0];
INVAR -1 <= ego.gaps___629___.i_agent_rear & ego.gaps___629___.i_agent_rear <= @{NONEGOS - 1}@.eval[0];

DEFINE
    @{ego.gaps___609___.lane_availability}@*.scalingVariable[distance] := case
        @{
           ego_lane_[i] | ego_lane_[i]@{[i]+1}@.eval[0] : ego.lane_@{[i] + 1}@.eval[0]_availability;
        }@*.for[[i], 0, @{NUMLANES - 2}@.eval]TRUE : 0;
    esac;

    @{ego.gaps___619___.lane_availability}@*.scalingVariable[distance] := case
        @{
           ego_lane_[i] : ego.lane_@{[i]}@.eval[0]_availability;
           ego_lane_[i]@{[i]+1}@.eval[0] : min(ego.lane_@{[i]}@.eval[0]_availability, ego.lane_@{[i]+1}@.eval[0]_availability);
        }@*.for[[i], 0, @{NUMLANES - 2}@.eval]TRUE : 0;
    esac;
    
    @{ego.gaps___629___.lane_availability}@*.scalingVariable[distance] := case
        @{
           ego_lane_[i] | ego_lane_@{[i]-1}@.eval[0][i] : ego.lane_@{[i] - 1}@.eval[0]_availability;
        }@*.for[[i], 1, @{NUMLANES - 1}@.eval]TRUE : 0;
    esac;


    @{
    @{
    ego.gaps___6[k]9___.v_front := case
        @{
        ego.gaps___6[k]9___.i_agent_front = [i] : veh___6[i]9___.v;
        }@*.for[[i], @{NONEGOS - 1}@.eval, 0, -1]
        TRUE : max_vel;  -- Max velocity is indicator of empty gap to the front, 0, to the rear.
    esac;
    }@.if[XVarGap[k]vfront]

    @{
    ego.gaps___6[k]9___.a_front := case
        @{
        ego.gaps___6[k]9___.i_agent_front = [i] : veh___6[i]9___.a;
        }@*.for[[i], @{NONEGOS - 1}@.eval, 0, -1]
        TRUE : 0;
    esac;
    }@.if[XVarGap[k]afront]

    @{
    ego.gaps___6[k]9___.turn_signals_front := case
	     @{
        ego.gaps___6[k]9___.i_agent_front = [i] : veh___6[i]9___.turn_signals;
        }@*.for[[i], @{NONEGOS - 1}@.eval, 0, -1]
        TRUE : ActionDir____CENTER;
    esac;
    }@.if[XVarGap[k]turnsignalsfront]

    @{
    ego.gaps___6[k]9___.v_rear := case
        @{
        ego.gaps___6[k]9___.i_agent_rear = [i] : veh___6[i]9___.v;
        }@*.for[[i], @{NONEGOS - 1}@.eval, 0, -1]
        TRUE : 0; -- Max velocity is indicator of empty gap to the front, 0, to the rear.
    esac;
    }@.if[XVarGap[k]vrear]

	}@**.for[[k], 0, 2]
)@
}@.if[@{NONEGOS <= THRESHOLD_FOR_USING_ASSIGNS_IN_GAP_STRUCTURE}@.eval]

--------------------------------------------------------
-- End: Gap Structure
--------------------------------------------------------

@{
--------------------------------------------------------
-- Prohibit lane changes for non-egos as long as ego is changing lanes
--------------------------------------------------------
@{
TRANS (next(ego.left_of_veh_[i]_lane)  & next(veh___6[i]9___.rel_pos) >= -veh_length & next(veh___6[i]9___.rel_pos) <= veh_length) -> !veh___6[i]9___.lane_move_up;
TRANS (next(ego.right_of_veh_[i]_lane) & next(veh___6[i]9___.rel_pos) >= -veh_length & next(veh___6[i]9___.rel_pos) <= veh_length) -> !veh___6[i]9___.lane_move_down;
}@.for[[i], 0, @{NONEGOS - 1}@.eval]
}@.if[@{DOUBLEMERGEPROTECTION}@.eval]

--------------------------------------------------------
-- Standing cars
--------------------------------------------------------
@{
TRANS next(veh___6[i]9___.prev_rel_pos) = veh___6[i]9___.rel_pos;
TRANS next(veh___6[i]9___.abs_pos) = veh___6[i]9___.abs_pos;
INVAR veh___6[i]9___.v = 0;
@{
TRANS next(veh___6[i]9___.lane_b@{[j]}@.eval[0]) = veh___6[i]9___.lane_b@{[j]}@.eval[0];
}@*.for[[j], 0, ~{{NUMLANES - 1}}~]
}@**.for[[i], 0, @{STANDINGCARSUPTOID}@.eval]

@{@{
   next(ego_lane_b[j]) := ego_lane_b[j];
}@*.for[[j], 0, @{NUMLANES - 1}@.eval]}@**.if[@{KEEPEGOFIXEDTOLANE}@.eval]

@{
@{EnvModel_DummyBP.tpl}@.include
}@.if[@{VIPER}@.eval]

@{EnvModel_Debug.tpl}@.include

)@
}@*****.if[@{EM_LESS}@.eval]