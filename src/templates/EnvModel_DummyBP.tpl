--------------------------------------------------------------------
--
-- Begin: Viper (by default generated from c++, not from this model)
-- Frozen version from 2023.
-- Caution: Scaling not implemented here. Use only with TIMESCALING = DISTANCESCALING = 1.
--
--------------------------------------------------------------------

VAR
    -- Viper variables
    flCond.cond0_lc_conditions_evaluated : boolean;
    flCond.cond1_lc_trigger_dist_reached : boolean;
    flCond.cond2_ego_faster_than_lead : boolean;
    flCond.cond3_lane_speed_allows_overtake : boolean; 
    flCond.cond4_lead_not_much_faster_than_ego : boolean;
    flCond.cond5_ax_acf_target_lane_same_or_higher : boolean;
    flCond.cond6_target_rear_vehicle_mild_braking : boolean;
    flCond.cond7_dist_to_target_front_vehicle_large_enough : boolean;
    flCond.cond8_dist_to_target_rear_vehicle_large_enough : boolean;
    flCond.cond10_t_action_ok : boolean;
    flCond.cond11_enable_lanechanges : boolean;
    flCond.cond12_current_lane_is_merge_lane : boolean;
    flCond.cond13_enable_merge_lanechange : boolean;
    flCond.cond14_shadow_agent_ahead : boolean;
    flCond.cond15_escape_slowlane_deadlock : boolean;
    flCond.cond16_forced_merge : boolean;
    flCond.cond18_driver_lanechange_request : boolean;
    flCond.cond19_navigation_system_request : boolean;
    flCond.cond20_reactive_motivation : boolean;
    flCond.cond20_reactive_timing : boolean;
    flCond.cond20_reactive_enablers : boolean; 
    flCond.cond20_reactive_conditions_fulfilled : boolean;
    flCond.cond22_safety_dynamic : boolean;
    flCond.cond22_safety_static : boolean;
    flCond.cond22_safety_conditions_fulfilled : boolean;
    flCond.cond24_external_conditions_fulfilled : boolean;

    
ASSIGN
    init(flCond.cond0_lc_conditions_evaluated) := FALSE;
    init(flCond.cond1_lc_trigger_dist_reached) := FALSE;
    init(flCond.cond2_ego_faster_than_lead) := FALSE;
    init(flCond.cond3_lane_speed_allows_overtake) := FALSE; 
    init(flCond.cond4_lead_not_much_faster_than_ego) := FALSE;
    init(flCond.cond5_ax_acf_target_lane_same_or_higher) := FALSE;
    init(flCond.cond6_target_rear_vehicle_mild_braking) := FALSE;
    init(flCond.cond7_dist_to_target_front_vehicle_large_enough) := FALSE;
    init(flCond.cond8_dist_to_target_rear_vehicle_large_enough) := FALSE;
    init(flCond.cond10_t_action_ok) := FALSE;
    init(flCond.cond11_enable_lanechanges) := TRUE;
    init(flCond.cond12_current_lane_is_merge_lane) := FALSE;
    init(flCond.cond13_enable_merge_lanechange) := TRUE;
    init(flCond.cond14_shadow_agent_ahead) := FALSE;
    init(flCond.cond15_escape_slowlane_deadlock) :=  FALSE;
    init(flCond.cond16_forced_merge) := FALSE;
    init(flCond.cond18_driver_lanechange_request) := FALSE;
    init(flCond.cond19_navigation_system_request) := FALSE;
    init(flCond.cond20_reactive_motivation) := FALSE;
    init(flCond.cond20_reactive_timing) := FALSE;
    init(flCond.cond20_reactive_enablers) := FALSE; 
    init(flCond.cond20_reactive_conditions_fulfilled) := FALSE;
    init(flCond.cond22_safety_dynamic) := FALSE;
    init(flCond.cond22_safety_static) := FALSE;
    init(flCond.cond22_safety_conditions_fulfilled) := FALSE;
    init(flCond.cond24_external_conditions_fulfilled) := FALSE;

    -- fast lane exists
    next(flCond.cond0_lc_conditions_evaluated) := case
        --max_lane_id - ego.on_lane >= 2: TRUE;
        ego_lane_max_free : TRUE;
        TRUE: FALSE;
    esac;

    -- fl condition 1: lc trigger distance reached
    next(flCond.cond1_lc_trigger_dist_reached) := ((ego.gaps___619___.s_dist_front <= (5 + (next(ego.gaps___619___.v_front) * 1) + max(2 * next(ego.gaps___619___.v_front) / 10, 1) + max(next(ego.v) - next(ego.gaps___619___.v_front), 0) * (ego.complete_lane_change_latest_after + params.turn_signal_duration) 
        + (max(next(ego.v) - next(ego.gaps___619___.v_front), 0) * max(next(ego.v) - next(ego.gaps___619___.v_front), 0) / 2) / 2)) & ((ego.gaps___619___.s_dist_front >= 3)));

    -- fl cond2: ego faster than lead vehicle: speed_this > speed_lead + delta
    next(flCond.cond2_ego_faster_than_lead) := case
        next(ego.gaps___619___.s_dist_front) <= max_ego_visibility_range : (next(ego.v) > next(ego.gaps___619___.v_front) + 2);
        TRUE: FALSE;
    esac;

    -- fl cond3: lane speed allows to accelerate and overtake the lead vehicle vMaxCurvature > speed_lead + delta
    next(flCond.cond3_lane_speed_allows_overtake) := case
        next(ego.gaps___619___.s_dist_front) <= max_ego_visibility_range : ego.max_vel > next(ego.v) + 1;
        TRUE: FALSE;
    esac; 

    -- fl cond4: lead not much faster than ego v_lead - v_ego < delta
    next(flCond.cond4_lead_not_much_faster_than_ego) := case
        next(ego.gaps___619___.s_dist_front) <= max_ego_visibility_range : next(ego.gaps___619___.v_front) - next(ego.v) <= 1;
        TRUE: FALSE;
    esac;

    -- fl cond5: ax_acf on target lane same or higher than on current lane ax_acf_target_lane >= ax_acf_current_lane
    -- refers to dynamically computer controller data that we do not have here
    next(flCond.cond5_ax_acf_target_lane_same_or_higher) := TRUE;

    -- fl cond 6: target lane rear vehicle does not need to brake hard s_dist_rear >= dist_min_fast_approaching
    next(flCond.cond6_target_rear_vehicle_mild_braking) := case
        next(ego.gaps___609___.s_dist_rear) <= max_ego_visibility_range: ego.gaps___609___.s_dist_rear >= (max(ego.gaps___609___.v_rear - ego.v, 0) * params.turn_signal_duration + (max(ego.gaps___609___.v_rear - ego.v, 0) * max(ego.gaps___609___.v_rear - ego.v, 0) * 3 / 10) + max(4* ego.v / 10, ego.min_dist_long));
        TRUE: TRUE;
    esac;

    -- fl cond 7: distance to target lane front vehicle large enough gap_target.s_dist_front > thres (check stationary and dynamic thres)
    next(flCond.cond7_dist_to_target_front_vehicle_large_enough) := case
        next(ego.gaps___609___.s_dist_front) <= max_ego_visibility_range: ego.gaps___609___.s_dist_front > max(max((max(ego.min_dist_long, (ego.v * 12 / 10)) - (max(ego.gaps___609___.v_front - ego.v, 0) * params.turn_signal_duration)), 0), (max(ego.v - ego.gaps___609___.v_front, 0)*max(ego.v - ego.gaps___609___.v_front, 0)) + max(ego.v * 10 / 12, ego.min_dist_long));
        TRUE: TRUE;
    esac;

    -- fl cond 8: distance to target lane rear vehicle large enough gap_target.s_dist_rear > thres
    next(flCond.cond8_dist_to_target_rear_vehicle_large_enough) := (next(ego.gaps___609___.s_dist_rear) > (max(5, (next(ego.v) * 4 / 10))));

    -- fl cond 10: time threshold after last action (check if needed) t_action >  thres
    next(flCond.cond10_t_action_ok) := ego.time_since_last_lc >= ego.min_time_between_lcs;

    -- fl cond 11: lanechanges enabled by parameter enable_lanechanges == true
    next(flCond.cond11_enable_lanechanges) := TRUE;

    -- fl cond 12: current lane is merge lane is_merge=true
    -- --> this condition is implemented, but not yet supported by the env model
    next(flCond.cond12_current_lane_is_merge_lane) := FALSE;

    -- fl cond 13: parameter "enable_merge_lanechange" is set to "true"
    next(flCond.cond13_enable_merge_lanechange) := TRUE;

    -- fl cond 14: shadow agent ahead
    -- TODO: not implemented, no shadow agents as currently only one non-ego vehicle
    next(flCond.cond14_shadow_agent_ahead) := FALSE;

    -- fl cond 15: escape slowlane deadlock
    next(flCond.cond15_escape_slowlane_deadlock) :=  case
        --max_lane_id - ego.on_lane >= 2 &
        ego_lane_max_free &
        ego.gaps___609___.s_dist_front <= max_ego_visibility_range : (next(ego.gaps___609___.s_dist_front) <= ((ego.min_dist_long + (next(ego.gaps___609___.v_front) * 1) + max(2 * next(ego.gaps___609___.v_front) / 10, 1) + max(next(ego.v) - next(ego.gaps___609___.v_front), 0) * (ego.complete_lane_change_latest_after + params.turn_signal_duration) 
        + (max(next(ego.v) - next(ego.gaps___609___.v_front), 0) * max(next(ego.v) - next(ego.gaps___609___.v_front), 0) / 2) / 2)))
            & ((ego.max_vel - next(ego.gaps___609___.v_front)) > 1)    
            & (next(ego.gaps___609___.v_front) - next(ego.v) <= 1);
        TRUE : FALSE;
    esac;

    -- fl cond 16: forced merge
    -- TODO: required values set outside the lane change conditions in the motion planner
    next(flCond.cond16_forced_merge) := FALSE;

    -- fl cond 18: driver trigger/confirmation
    next(flCond.cond18_driver_lanechange_request) := {TRUE, FALSE};

    next(flCond.cond19_navigation_system_request) := FALSE;

    -- combined reactive conditions
    next(flCond.cond20_reactive_motivation) := (  (  next(flCond.cond1_lc_trigger_dist_reached) & (    
        next(flCond.cond2_ego_faster_than_lead) | (   next(flCond.cond3_lane_speed_allows_overtake)    & next(flCond.cond4_lead_not_much_faster_than_ego) )
         ) & next(flCond.cond5_ax_acf_target_lane_same_or_higher) 
      )| next(flCond.cond14_shadow_agent_ahead)| next(flCond.cond16_forced_merge) | next(flCond.cond15_escape_slowlane_deadlock) );

    next(flCond.cond20_reactive_timing) := ( next(flCond.cond10_t_action_ok) | next(flCond.cond14_shadow_agent_ahead) | next(flCond.cond16_forced_merge) );

    next(flCond.cond20_reactive_enablers) := next(flCond.cond11_enable_lanechanges); 

    next(flCond.cond20_reactive_conditions_fulfilled) := next(flCond.cond20_reactive_motivation) & next(flCond.cond20_reactive_timing) & next(flCond.cond20_reactive_enablers);

    -- combined safety conditions
    next(flCond.cond22_safety_dynamic) := next(flCond.cond6_target_rear_vehicle_mild_braking);

    next(flCond.cond22_safety_static) := next(flCond.cond7_dist_to_target_front_vehicle_large_enough) & next(flCond.cond8_dist_to_target_rear_vehicle_large_enough);

    next(flCond.cond22_safety_conditions_fulfilled) := next(flCond.cond22_safety_dynamic) & next(flCond.cond22_safety_static);

    next(flCond.cond24_external_conditions_fulfilled) := (next(flCond.cond18_driver_lanechange_request) | next(flCond.cond19_navigation_system_request));

    ego.flCond_full := (flCond.cond20_reactive_conditions_fulfilled | flCond.cond24_external_conditions_fulfilled) & flCond.cond22_safety_conditions_fulfilled;

--------------------------------------------------------
-- End: Viper
--------------------------------------------------------
