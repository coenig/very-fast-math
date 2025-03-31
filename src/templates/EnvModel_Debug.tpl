----------------
-- DEBUG STUFF
----------------

@{
VAR
  @{debug.ego.gaps___609___.lane_availability}@*.scalingVariable[distance] : integer;
  @{debug.ego.gaps___619___.lane_availability}@*.scalingVariable[distance] : integer;
  @{debug.ego.gaps___629___.lane_availability}@*.scalingVariable[distance] : integer;

  @{debug.ego.a}@*.scalingVariable[acceleration] : integer;
  
  @{debug.gaps___609___.v_front}@*.scalingVariable[velocity] : integer;
  @{debug.gaps___609___.a_front}@*.scalingVariable[acceleration] : integer;
  debug.gaps___609___.turn_signals_front : {ActionDir____LEFT, ActionDir____CENTER, ActionDir____RIGHT};

  @{debug.gaps___619___.v_front}@*.scalingVariable[velocity] : integer;
  @{debug.gaps___619___.a_front}@*.scalingVariable[acceleration] : integer;

  @{debug.gaps___629___.v_front}@*.scalingVariable[velocity] : integer;
  @{debug.gaps___629___.a_front}@*.scalingVariable[acceleration] : integer;
  debug.gaps___629___.turn_signals_front : {ActionDir____LEFT, ActionDir____CENTER, ActionDir____RIGHT};

  @{debug.allowed_ego_a_front_center}@*.scalingVariable[acceleration] : integer;
  @{debug.allowed_ego_a_front_left}@*.scalingVariable[acceleration] : integer;
  @{debug.allowed_ego_a_front_right}@*.scalingVariable[acceleration] : integer;
  
  @{debug.rlc}@*.scalingVariable[distance] : integer;

  debug.ego_pressured_from_ahead_on_left_lane : boolean;
  debug.ego_pressured_from_ahead_on_own_lane : boolean;
  debug.ego_pressured_from_ahead_on_right_lane : boolean;
  debug.crash : boolean;

  @{
  debug.ego.right_of_veh_[i]_lane : boolean;
  debug.ego.left_of_veh_[i]_lane : boolean;
  }@.for[[i], 0, @{NONEGOS - 1}@.eval]
  
ASSIGN
  init(debug.ego.gaps___609___.lane_availability) := ego.gaps___609___.lane_availability;
  init(debug.ego.gaps___619___.lane_availability) := ego.gaps___619___.lane_availability;
  init(debug.ego.gaps___629___.lane_availability) := ego.gaps___629___.lane_availability;
  init(debug.rlc) := rlc;

  init(debug.ego.a) := ego.a;
  init(debug.gaps___609___.turn_signals_front) := ActionDir____CENTER;
  init(debug.gaps___629___.turn_signals_front) := ActionDir____CENTER;
  init(debug.gaps___609___.v_front) := 0;
  init(debug.gaps___619___.v_front) := 0;
  init(debug.gaps___629___.v_front) := 0;
  init(debug.gaps___609___.a_front) := 0;
  init(debug.gaps___619___.a_front) := 0;
  init(debug.gaps___629___.a_front) := 0;
  init(debug.allowed_ego_a_front_center) := 0;
  init(debug.allowed_ego_a_front_left) := 0;
  init(debug.allowed_ego_a_front_right) := 0;
  init(debug.crash) := crash;

  init(debug.ego_pressured_from_ahead_on_left_lane)                   := ego_pressured_from_ahead_on_left_lane;
  init(debug.ego_pressured_from_ahead_on_own_lane)                    := ego_pressured_from_ahead_on_own_lane;
  init(debug.ego_pressured_from_ahead_on_right_lane)                  := ego_pressured_from_ahead_on_right_lane;

  @{
  init(debug.ego.right_of_veh_[i]_lane) := ego.right_of_veh_[i]_lane;
  init(debug.ego.left_of_veh_[i]_lane) := ego.left_of_veh_[i]_lane;
  next(debug.ego.right_of_veh_[i]_lane) := next(ego.right_of_veh_[i]_lane);
  next(debug.ego.left_of_veh_[i]_lane) := next(ego.left_of_veh_[i]_lane);
  }@.for[[i], 0, @{NONEGOS - 1}@.eval]

  next(debug.ego.gaps___609___.lane_availability) := next(ego.gaps___609___.lane_availability);
  next(debug.ego.gaps___619___.lane_availability) := next(ego.gaps___619___.lane_availability);
  next(debug.ego.gaps___629___.lane_availability) := next(ego.gaps___629___.lane_availability);

  next(debug.ego.a) := next(ego.a);
  next(debug.gaps___609___.turn_signals_front) := next(ego.gaps___609___.turn_signals_front);
  next(debug.gaps___629___.turn_signals_front) := next(ego.gaps___629___.turn_signals_front);
  next(debug.gaps___609___.v_front) := next(ego.gaps___609___.v_front);
  next(debug.gaps___619___.v_front) := next(ego.gaps___619___.v_front);
  next(debug.gaps___629___.v_front) := next(ego.gaps___629___.v_front);
  next(debug.gaps___609___.a_front) := next(ego.gaps___609___.a_front);
  next(debug.gaps___619___.a_front) := next(ego.gaps___619___.a_front);
  next(debug.gaps___629___.a_front) := next(ego.gaps___629___.a_front);
  next(debug.allowed_ego_a_front_center) := next(allowed_ego_a_front_center);
  next(debug.allowed_ego_a_front_left)   := next(allowed_ego_a_front_left);
  next(debug.allowed_ego_a_front_right)  := next(allowed_ego_a_front_right);
  next(debug.ego_pressured_from_ahead_on_left_lane)  := next(ego_pressured_from_ahead_on_left_lane);
  next(debug.ego_pressured_from_ahead_on_own_lane)  := next(ego_pressured_from_ahead_on_own_lane);
  next(debug.ego_pressured_from_ahead_on_right_lane)  := next(ego_pressured_from_ahead_on_right_lane);
  next(debug.crash)  := next(crash);
  next(debug.rlc)  := next(rlc);
}@**.if[@{DEBUG}@.eval]

----------------
-- EO DEBUG STUFF
----------------
