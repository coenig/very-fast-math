@{

-------------------------------------------------------------------------------
--
-- Begin: Ego Env Model Part (generate once -- or none, in case EGOLESS = true)
--
-------------------------------------------------------------------------------

VAR
    -- Ego physical state
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
    ego.gaps___629___.i_agent_rear : empty_gap_indicator..empty_gap_indicator;

    tar_dir : {ActionDir____LEFT, ActionDir____CENTER, ActionDir____RIGHT};

@{
    #ERROR: ASSIGN structure for gaps not supported anymore since 2024-06-20.
}@******.nil

INVAR tar_dir = ActionDir____LEFT; -- TODO: This is only needed as interface towards BP. Remove when better solution comes up.

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
    ego_lane_crossing := FALSE @{| ego_lane_@{[j]-1}@.eval[0][j]}@*.for[[j], 1, @{NUMLANES - 1}@.eval];
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

    -- "Soft" counts half a lane as neighboring, too. (Obsolete "regular" mode which counted only exactly one lane distance has been removed.)
    -- TODO: Case missing where ego is between far left/right lane and the one next to it, and the other car is on the resp. extreme lane.

    @{
    ego.right_of_veh_[i]_lane := FALSE
       @{| (ego_lane_@{[j]-1}@.eval[0] & veh___6[i]9___.lane_[j]) | (ego_lane_@{[j]-1}@.eval[0] & veh___6[i]9___.lane_~{[j]-1\0}~[j])
       }@*.for[[j], 1, @{NUMLANES - 1}@.eval, 1, | (ego_lane_~{[j]-2\0}~~{[j]-1\0}~ & veh___6[i]9___.lane_~{[j]-1\0}~[j]) | (ego_lane_~{[j]-2\0}~~{[j]-1\0}~ & veh___6[i]9___.lane_~{[j]-1\0}~)];

    ego.left_of_veh_[i]_lane := FALSE
       @{| (ego_lane_[j] & veh___6[i]9___.lane_@{[j]-1}@.eval[0]) | (ego_lane_[j] & veh___6[i]9___.lane_@{[j]-1}@.eval[0][j])
       }@*.for[[j], 1, @{NUMLANES - 1}@.eval, 1, | (ego_lane_~{[j]-1\0}~~{[j]\0}~ & veh___6[i]9___.lane_~{[j]-2\0}~~{[j]-1\0}~) | (ego_lane_~{[j]-1\0}~~{[j]\0}~ & veh___6[i]9___.lane_~{[j]-1\0}~)];

   ego.same_lane_as_veh_[i] := (FALSE
      @{| ((ego_lane_b[j] & veh___6[i]9___.lane_b[j]) @{& !(ego_lane_b@{[j]-1}@.eval[0] & veh___6[i]9___.lane_b@{[j]+1}@.eval[0]) & !(ego_lane_b@{[j]+1}@.eval[0] & veh___6[i]9___.lane_b@{[j]-1}@.eval[0])}@.if[@{[j] > 0 && [j] < NUMLANES - 1}@.eval] )
      }@*.for[[j], 0, @{NUMLANES - 1}@.eval]
   );

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

	ego.crash_with_veh_[i] := ego.same_lane_as_veh_[i] & (veh___6[i]9___.rel_pos >= -veh_length & veh___6[i]9___.rel_pos <= veh_length);
	ego.blamable_crash_with_veh_[i] := ego.same_lane_as_veh_[i] & (veh___6[i]9___.rel_pos >= 0 & veh___6[i]9___.rel_pos <= veh_length);

   ego_pressured_by_vehicle_[i] := ego.same_lane_as_veh_[i] 
        @{| (veh___6[i]9___.lc_direction = ActionDir____RIGHT & ego.right_of_veh_[i]_lane & veh___6[i]9___.change_lane_now = 1)
        | (veh___6[i]9___.lc_direction = ActionDir____LEFT & ego.left_of_veh_[i]_lane   & veh___6[i]9___.change_lane_now = 1)}@******.if[@{!SIMPLE_LC}@.eval];

ego_pressured_by_vehicle_[i]_from_behind := ego_pressured_by_vehicle_[i] & (veh___6[i]9___.prev_rel_pos < 0);

-- minimum_dist_to_veh_[i] := (veh___6[i]9___.rel_pos + ego_v_minus_veh_[i]_v_square / (2* abs(a_min))); -- This was another way of doing it, though not quite accurate; turns out, it is also slower.

@{minimum_dist_to_veh_[i]}@.scalingVariable[distance] := veh___6[i]9___.rel_pos - square_of_veh_v_[i] / (2*a_min) + square_of_veh_v_0 / (2*a_min);


   }@**.for[[i], 1, @{NONEGOS - 1}@.eval]

crash := FALSE@{@{}@.space| ego.crash_with_veh_[i]}@*.for[[i], 1, @{NONEGOS - 1}@.eval];
blamable_crash := FALSE@{@{}@.space| ego.blamable_crash_with_veh_[i]}@*.for[[i], 1, @{NONEGOS - 1}@.eval];

-- TODO: Formerly INVAR, and "=" instead of ":=" - Check if this is correct!
ego.has_close_vehicle_on_left_left_lane := (FALSE @{@{ | ego.close_to_vehicle_[i]_on_left_left_lane}@.for[[i], 1, @{NONEGOS - 1}@.eval, 1]}@******.if[@{FARMERGINGCARS}@.eval]);     -- For some reason the "FALSE |"...
ego.has_close_vehicle_on_right_right_lane := (FALSE @{@{ | ego.close_to_vehicle_[i]_on_right_right_lane}@.for[[i], 1, @{NONEGOS - 1}@.eval, 1]}@******.if[@{FARMERGINGCARS}@.eval]); -- ...seems to be crucial for runtime.

@{
-- check that chosen long acceleration of other vehicle is indeed safe
INVAR
    ego_pressured_by_vehicle_[i]_from_behind ->
            (minimum_dist_to_veh_[i] <= min_dist_long & (veh___6[i]9___.rel_pos < -veh_length));

-- Make sure ego and vehicle [i] don't collide in the initial state.
INIT abs(veh___6[i]9___.rel_pos) > veh_length | !ego.same_lane_as_veh_[i];

INVAR -- Ego may not "jump" over non-ego cars.
    !(ego.same_lane_as_veh_[i] & (veh___6[i]9___.prev_rel_pos < 0) & (veh___6[i]9___.rel_pos >= 0)) &
    !(ego.same_lane_as_veh_[i] & (veh___6[i]9___.prev_rel_pos > 0) & (veh___6[i]9___.rel_pos <= 0));
}@**.for[[i], 1, @{NONEGOS - 1}@.eval]
    


INVAR
    ego_lane_single | ego_lane_crossing;

INIT ego_lane_single;


-- TODO: Needed?
@{
INVAR
    (!(ego.abCond_full & ego.mode = ActionType____LANE_FOLLOWING));  -- only abort when a lane change is ongoing
}@******.if[@{ABORTREVOKE}@.eval]

INVAR
    (0 <= ego.v & ego.v <= ego.max_vel) &
    (ego.min_accel <= ego.a & ego.a <= ego.max_accel);

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
}@******.nil

    init(ego.lc_direction) := ActionDir____CENTER;
    init(ego.mode) := ActionType____LANE_FOLLOWING;
    init(ego_timer) := -1;
    init(ego.change_lane_now) := 0;
  
    init(ego.time_since_last_lc) := ego.min_time_between_lcs;       -- init with max value such that lane change is immediately allowed after start

    -- ego.v 
    next(ego.v) := min(max(ego.v + ego.a, 0), ego.max_vel);


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
}@******.if[@{BRAKEINHIBITION}@.eval]

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
}@******.if[@{LONGCONTROL}@.eval]

@{
-- Prohibit continuing LC in case of abort condition already active.
TRANS (next(ego.mode = ActionType____LANECHANGE_ABORT) | ego.mode = ActionType____LANECHANGE_ABORT | next(ego.abCond_full) | ego.abCond_full) -> (!ego_lane_move_up & !ego_lane_move_down);
}@******.if[@{ABORTREVOKE}@.eval]

ASSIGN
    @{
    @{@(@(LEFT)@)@}@*.if[@{LEFTLC}@.eval]
    @{@(@(RIGHT)@)@}@*.if[@{RIGHTLC}@.eval]
    }@.setScriptVar[possible_lc_modes]

    next(ego.lc_direction) := case
        @{ego.mode = ActionType____LANE_FOLLOWING & ego.flCond_full: ActionDir____LEFT;}@.if[@{LEFTLC}@.eval]
        @{ego.mode = ActionType____LANE_FOLLOWING & ego.slCond_full: ActionDir____RIGHT;}@.if[@{RIGHTLC}@.eval]
        next(ego.mode) = ActionType____LANE_FOLLOWING: ActionDir____CENTER;
        TRUE: ego.lc_direction;
    esac;

    next(ego.mode) := case
    @{
        (FALSE @{| ego.mode = ActionType____LANECHANGE_[DIR]}@.for[[DIR], @{possible_lc_modes}@.scriptVar]) & (ego.abCond_full | next(ego.abCond_full)): ActionType____LANECHANGE_ABORT;
    }@******.if[@{!ABORTREVOKE}@.eval]
        @{ego.mode = ActionType____LANE_FOLLOWING & ego.flCond_full: ActionType____LANECHANGE_LEFT;}@.if[@{LEFTLC}@.eval]
        @{ego.mode = ActionType____LANE_FOLLOWING & ego.slCond_full: ActionType____LANECHANGE_RIGHT;}@.if[@{RIGHTLC}@.eval]
        @{ego.mode = ActionType____LANECHANGE_LEFT & ego_timer = ego.complete_lane_change_latest_after: ActionType____LANE_FOLLOWING;}@.if[@{LEFTLC}@.eval]
        @{ego.mode = ActionType____LANECHANGE_RIGHT & ego_timer = ego.complete_lane_change_latest_after: ActionType____LANE_FOLLOWING;}@.if[@{RIGHTLC}@.eval]
        @{ego.mode = ActionType____LANECHANGE_LEFT & ego_timer >= ego.complete_lane_change_earliest_after  & ego_timer < ego.complete_lane_change_latest_after : {ActionType____LANE_FOLLOWING, ActionType____LANECHANGE_LEFT};}@.if[@{LEFTLC}@.eval]
        @{ego.mode = ActionType____LANECHANGE_RIGHT & ego_timer >= ego.complete_lane_change_earliest_after  & ego_timer < ego.complete_lane_change_latest_after : {ActionType____LANE_FOLLOWING, ActionType____LANECHANGE_RIGHT};}@.if[@{RIGHTLC}@.eval]
    @{
        (FALSE @{| ego.mode = ActionType____LANECHANGE_[DIR]}@.for[[DIR], @{possible_lc_modes}@.scriptVar]) & ego.abCond_full: ActionType____LANECHANGE_ABORT;
    }@******.if[@{ABORTREVOKE}@.eval]
        ego.mode = ActionType____LANECHANGE_ABORT & ego_timer = 0: ActionType____LANE_FOLLOWING;
        TRUE: ego.mode;
    esac;

    next(ego_timer) := case                                                                 -- IMPORTANT: Order of cases is exploitet here!!
        ego_timer = -1 & ego.mode = ActionType____LANE_FOLLOWING & (FALSE @{| next(ego.mode) = ActionType____LANECHANGE_[DIR]}@.for[[DIR], @{possible_lc_modes}@.scriptVar]) : 0;         -- on the rising edge, set time to 0 and activate
        ego_timer >= 0 & next(ego.mode) = ActionType____LANE_FOLLOWING: -1;                                  -- deactivate counter once lane change is complete 
        ego_timer >= 0 & (FALSE @{| ego.mode = ActionType____LANECHANGE_[DIR]}@.for[[DIR], @{possible_lc_modes}@.scriptVar]) & next(ego.mode) = ActionType____LANECHANGE_ABORT & ego_timer >= params.turn_signal_duration: ego_timer - params.turn_signal_duration; -- going back takes the same time as going forward, but do not use turn signal duration for abort
        ego_timer > 0 & ego.mode = ActionType____LANECHANGE_ABORT: ego_timer - 1;                        -- count down back to zero while aborting
        ego_timer >= 0 & ego_timer < ego.complete_lane_change_latest_after : ego_timer + 1;   -- while we are still changing lanes, increment counter
        TRUE: ego_timer;                                                                     -- keep counter at its value in all other cases
    esac;

    next(ego.time_since_last_lc) := case
        (@{ego.mode = ActionType____LANECHANGE_[DIR] | }@.for[[DIR], @{possible_lc_modes}@.scriptVar] ego.mode = ActionType____LANECHANGE_ABORT) & next(ego.mode) = ActionType____LANE_FOLLOWING : 0;            -- activate timer when lane change finishes
        ego.time_since_last_lc >= 0 & ego.time_since_last_lc < ego.min_time_between_lcs: ego.time_since_last_lc + 1;    -- increment until threshold is reached, saturate at the threshold (-> else condition)
        ego.mode = ActionType____LANE_FOLLOWING & (FALSE @{| next(ego.mode) = ActionType____LANECHANGE_[DIR]}@.for[[DIR], @{possible_lc_modes}@.scriptVar]): -1;           -- deactivate timer when new lane change starts
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
    -- >>> Car [i] <<<
    veh___6[i]9___.is_visible := abs(veh___6[i]9___.rel_pos) <= max_ego_visibility_range;
    veh___6[i]9___.is_in_front := veh___6[i]9___.rel_pos >= 0;
    veh___6[i]9___.is_behind := veh___6[i]9___.rel_pos < 0;
    veh___6[i]9___.is_visible_in_front := veh___6[i]9___.is_in_front & veh___6[i]9___.is_visible;
    veh___6[i]9___.is_visible_behind := veh___6[i]9___.is_behind & veh___6[i]9___.is_visible;
	
}@.for[[i], 1, @{NONEGOS - 1}@.eval]


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

@{@(
-- GAPS[0] ==> Left of Ego
-- FRONT
INVAR
@{(ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front & (ego.gaps___609___.s_dist_front = veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
(ego.gaps___609___.s_dist_front = max_ego_visibility_range + 1);

INVAR
@{((ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front) -> (ego.gaps___609___.s_dist_front <= veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
((@{!(ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front) &
}@.for[[i], 1, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___609___.s_dist_front = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___609___.s_dist_front = veh___6[i]9___.rel_pos - veh_length) & ego.right_of_veh_[i]_lane) <-> ego.gaps___609___.i_agent_front = [i]) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
TRUE;

-- REAR
INVAR
@{(ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind & (ego.gaps___609___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
(ego.gaps___609___.s_dist_rear = max_ego_visibility_range + 1);

INVAR
@{((ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind) -> (ego.gaps___609___.s_dist_rear <= -veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
((@{!(ego.right_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind) &
}@.for[[i], 1, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___609___.s_dist_rear = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___609___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length) & ego.right_of_veh_[i]_lane) <-> ego.gaps___609___.i_agent_rear = [i]) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
TRUE;
)@
@(
INVAR ego.gaps___609___.i_agent_rear = empty_gap_indicator & ego.gaps___609___.i_agent_front = empty_gap_indicator;
INVAR ego.gaps___609___.s_dist_front = max_ego_visibility_range + 1;
INVAR ego.gaps___609___.s_dist_rear = max_ego_visibility_range + 1;
)@
}@******.if[@{CALCULATE_LEFT_GAP}@.eval]

@{@(
-- GAPS[1] ==> On Ego lane
-- FRONT
INVAR
@{(ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_in_front & (ego.gaps___619___.s_dist_front = veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
(ego.gaps___619___.s_dist_front = max_ego_visibility_range + 1);

INVAR
@{((ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_in_front) -> (ego.gaps___619___.s_dist_front <= veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
((@{!(ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_in_front) &
}@.for[[i], 1, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___619___.s_dist_front = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___619___.s_dist_front = veh___6[i]9___.rel_pos - veh_length) & ego.same_lane_as_veh_[i]) <-> ego.gaps___619___.i_agent_front = [i]) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
TRUE;

--REAR
INVAR
@{(ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_behind & (ego.gaps___619___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
(ego.gaps___619___.s_dist_rear = max_ego_visibility_range + 1);

INVAR
@{((ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_behind) -> (ego.gaps___619___.s_dist_rear <= -veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
((@{!(ego.same_lane_as_veh_[i] & veh___6[i]9___.is_visible_behind) &
}@.for[[i], 1, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___619___.s_dist_rear = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___619___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length) & ego.same_lane_as_veh_[i]) <-> ego.gaps___619___.i_agent_rear = [i]) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
TRUE;
)@
@(
INVAR ego.gaps___619___.i_agent_rear = empty_gap_indicator & ego.gaps___619___.i_agent_front = empty_gap_indicator;
INVAR ego.gaps___619___.s_dist_front = max_ego_visibility_range + 1;
INVAR ego.gaps___619___.s_dist_rear = max_ego_visibility_range + 1;
)@
}@******.if[@{CALCULATE_CENTER_GAP}@.eval]


@{@(
-- GAPS[2] ==> Right of Ego
-- FRONT
INVAR
@{(ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front & (ego.gaps___629___.s_dist_front = veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
(ego.gaps___629___.s_dist_front = max_ego_visibility_range + 1);

INVAR
@{((ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front) -> (ego.gaps___629___.s_dist_front <= veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
((@{!(ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_in_front) &
}@.for[[i], 1, @{NONEGOS - 1}@.eval] TRUE) <-> (ego.gaps___629___.s_dist_front = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___629___.s_dist_front = veh___6[i]9___.rel_pos - veh_length) & ego.left_of_veh_[i]_lane) <-> ego.gaps___629___.i_agent_front = [i]) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
TRUE;

--REAR
@{
INVAR
@{(ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind & (ego.gaps___629___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length)) | 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
(ego.gaps___629___.s_dist_rear = max_ego_visibility_range + 1);

INVAR
@{((ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind) -> (ego.gaps___629___.s_dist_rear <= -veh___6[i]9___.rel_pos - veh_length)) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
((@{!(ego.left_of_veh_[i]_lane & veh___6[i]9___.is_visible_behind) &
}@.for[[i], 1, @{NONEGOS - 1}@.eval] TRUE) -> (ego.gaps___629___.s_dist_rear = max_ego_visibility_range + 1));

INVAR
@{(((ego.gaps___629___.s_dist_rear = -veh___6[i]9___.rel_pos - veh_length) & ego.left_of_veh_[i]_lane) <-> ego.gaps___629___.i_agent_rear = [i]) & 
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
TRUE;
}@*****.if[@{CALCULATE_RIGHT_GAP_REAR}@.eval]
)@
@(
INVAR ego.gaps___629___.i_agent_rear = empty_gap_indicator & ego.gaps___629___.i_agent_front = empty_gap_indicator;
INVAR ego.gaps___629___.s_dist_front = max_ego_visibility_range + 1;
INVAR ego.gaps___629___.s_dist_rear = max_ego_visibility_range + 1;
)@
}@******.if[@{CALCULATE_RIGHT_GAP}@.eval]

INVAR empty_gap_indicator <= ego.gaps___609___.i_agent_front & ego.gaps___609___.i_agent_front <= @{NONEGOS - 1}@.eval[0];
INVAR empty_gap_indicator <= ego.gaps___619___.i_agent_front & ego.gaps___619___.i_agent_front <= @{NONEGOS - 1}@.eval[0];
INVAR empty_gap_indicator <= ego.gaps___629___.i_agent_front & ego.gaps___629___.i_agent_front <= @{NONEGOS - 1}@.eval[0];
INVAR empty_gap_indicator <= ego.gaps___609___.i_agent_rear & ego.gaps___609___.i_agent_rear <= @{NONEGOS - 1}@.eval[0];
INVAR empty_gap_indicator <= ego.gaps___619___.i_agent_rear & ego.gaps___619___.i_agent_rear <= @{NONEGOS - 1}@.eval[0];
INVAR empty_gap_indicator <= ego.gaps___629___.i_agent_rear & ego.gaps___629___.i_agent_rear <= @{NONEGOS - 1}@.eval[0];

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
        }@*.for[[i], @{NONEGOS - 1}@.eval, 1, -1]
        TRUE : max_vel;  -- Max velocity is indicator of empty gap to the front, 0, to the rear.
    esac;
    }@.if[XVarGap[k]vfront]

    @{
    ego.gaps___6[k]9___.a_front := case
        @{
        ego.gaps___6[k]9___.i_agent_front = [i] : veh___6[i]9___.a;
        }@*.for[[i], @{NONEGOS - 1}@.eval, 1, -1]
        TRUE : 0;
    esac;
    }@.if[XVarGap[k]afront]

    @{
    ego.gaps___6[k]9___.turn_signals_front := case
        @{
	     @{
        ego.gaps___6[k]9___.i_agent_front = [i] : veh___6[i]9___.turn_signals;
        }@*.for[[i], @{NONEGOS - 1}@.eval, 1, -1]
        }@******.if[@{!SIMPLE_LC}@.eval]
        TRUE : ActionDir____CENTER;
    esac;
    }@.if[XVarGap[k]turnsignalsfront]

    @{
    ego.gaps___6[k]9___.v_rear := case
        @{
        ego.gaps___6[k]9___.i_agent_rear = [i] : veh___6[i]9___.v;
        }@*.for[[i], @{NONEGOS - 1}@.eval, 1, -1]
        TRUE : 0; -- Max velocity is indicator of empty gap to the front, 0, to the rear.
    esac;
    }@.if[XVarGap[k]vrear]

	}@**.for[[k], 0, 2]
)@
}@******.if[@{NONEGOS < THRESHOLD_FOR_USING_ASSIGNS_IN_GAP_STRUCTURE}@.eval]

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
}@.for[[i], 1, @{NONEGOS - 1}@.eval]
}@******.if[@{DOUBLEMERGEPROTECTION}@.eval]

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
}@**.for[[i], 1, @{STANDINGCARSUPTOID}@.eval]

@{@{
   next(ego_lane_b[j]) := ego_lane_b[j];
}@*.for[[j], 0, @{NUMLANES - 1}@.eval]}@******.if[@{KEEPEGOFIXEDTOLANE}@.eval]

 -- Synch EGO with veh___609___.
@{
INVAR ego_lane_b[j] = veh___609___.lane_b[j];
}@.for[[j], 0, @{NUMLANES - 1}@.eval]
INVAR ego.v = veh___609___.v;
INVAR ego.a = veh___609___.a;
INVAR ego.abs_pos = veh___609___.abs_pos;
 -- EO Synch EGO with veh___609___.

@{
@{EnvModel_DummyBP.tpl}@.include
}@******.if[@{VIPER}@.eval]

@{EnvModel_Debug.tpl}@.include

}@******.if[@{!(EGOLESS)}@.eval]

---------------------------------------------------------------------
-- Here comes logic which is independent of EGO-less or EGO-full mode
---------------------------------------------------------------------
-- TODO: The ego not driving on the green stuff should be probably here, as well, once the section logic is there.
-- TODO: The ego lane logic (without lane change) should be probably here, as well.

VAR
   -- "cond" variables are in ego-less mode only there to mock the interface towards the BP
   ego.flCond_full : boolean; -- conditions for lane change to fast lane (lc allowed and desired)
   ego.slCond_full : boolean; -- conditions for lane change to slow lane (lc allowed and desired)
   ego.abCond_full : boolean; -- conditions for abort of lane change 
   -- EO "cond" variables are in ego-less mode only there to mock the interface towards the BP

   @{ego.abs_pos}@*.scalingVariable[distance] : integer;

@{FROZENVAR}@******.if[@{(EGOLESS)}@.eval]
   @{ego.v}@*.scalingVariable[velocity] : 0 .. ego.max_vel; -- @{2}@.velocityWorldToEnvModelConst;

INIT ego.abs_pos = 0;

ASSIGN
   next(ego.abs_pos) := ego.abs_pos + ego.v;


---------------------------------------------------------------------
-- EO logic which is independent of EGO-less or EGO-full mode
---------------------------------------------------------------------
