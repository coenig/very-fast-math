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

@{
    @{ego.max_vel}@*.velocityWorldToEnvModelDef[MAXSPEEDEGO];
    @{ego.min_dist_long}@*.scalingVariable[distance] := veh_length;
    @{ego.following_dist}@*.scalingVariable[distance] := max(@{2}@.timeWorldToEnvModelConst * ego.v, ego.min_dist_long);  -- dynamic following distance according to 2s rule (= THW)
    @{ego.min_accel}@*.accelerationWorldToEnvModelDef[MINACCELEGO];
    @{ego.max_accel}@*.accelerationWorldToEnvModelDef[MAXACCELEGO];
}@**.if[@{!(EGOLESS)}@.eval]

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
