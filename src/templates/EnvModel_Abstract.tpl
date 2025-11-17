MODULE main
  VAR env_model : EnvModel;

MODULE EnvModel
-- TODO[AB]: This is a subset of the Constants.tpl file!
DEFINE
    @{min_time_between_lcs}@*.timeWorldToEnvModelDef[MIN_TIME_BETWEEN_LANECHANGES]; -- after finisihing one lc, how much time needs to pass before the next one may be started
    @{params.turn_signal_duration}@*.timeWorldToEnvModelDef[2]; -- turn signals will be on for this amount of time -- TODO: Connection via vfm-aka not working, has to be investigated.
    -- total lc duration shall be 5 s according to code documentation
    -- src lane is left 1s or 2s after end of turn signal duration, tgt lane is reached after 5s
    @{ego.leave_src_lane_earliest_after}@*.timeWorldToEnvModelDef[1] + params.turn_signal_duration;  -- 1 s after end of turn signal duration
    @{ego.leave_src_lane_latest_after}@*.timeWorldToEnvModelDef[2] + params.turn_signal_duration;    -- 2 s after end of turn signal duration
    @{ego.complete_lane_change_earliest_after}@*.timeWorldToEnvModelDef[2] + params.turn_signal_duration;
    @{ego.complete_lane_change_latest_after}@*.timeWorldToEnvModelDef[2] + params.turn_signal_duration;
    -- it seems that there is no parameter given for the duration of the abort, assume same duration to roll back that it took to get to the current pos

    @{leave_src_lane_earliest_after}@*.timeWorldToEnvModelDef[1];             -- earliest point in time where the vehicle may cross the lane border, i.e., after this transition the vehicle occupies two lanes 
    @{leave_src_lane_latest_after}@*.timeWorldToEnvModelDef[5];
    @{complete_lane_change_earliest_after}@*.timeWorldToEnvModelDef[1];       -- earliest point in time where the vehicle is entirely on its target lane, i.e., after this transition the vehicle only occupies one lane
    @{complete_lane_change_latest_after}@*.timeWorldToEnvModelDef[7];
    @{abort_lane_change_complete_earliest_after}@*.timeWorldToEnvModelDef[1]; -- earliest point in time where the vehicle is entirely back on its source lane after a lane change abort
    @{abort_lane_change_complete_latest_after}@*.timeWorldToEnvModelDef[3];
    @{ego.min_time_between_lcs}@*.timeWorldToEnvModelDef[2];                      -- after finisihing one lc, how much time needs to pass before the next one may be started
@{EnvModel_Abstract_Sections.tpl}@*******.include
@{EnvModel_Abstract_Behavior_Nonego.tpl}@********.include
