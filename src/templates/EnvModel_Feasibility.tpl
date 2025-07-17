@{

DEFINE
   -- Feasibility calculation according to R79
   -- Rear-left Criticality (RLC)
   @{rlc_v_rear}@*.scalingVariable[velocity] := ego.gaps___609___.v_rear;
   @{rlc_v_acsf}@*.scalingVariable[velocity] := ego.v;
   @{rlc_v_delta}@*.scalingVariable[velocity] := rlc_v_rear - rlc_v_acsf;
   @{rlc_tB}@*.scalingVariable[time] := @{0.4}@.timeWorldToEnvModelConst;
   @{rlc_a}@*.scalingVariable[acceleration] := @{3}@.accelerationWorldToEnvModelConst; -- Maximal braking acceleration expected of other vehicles.
   @{rlc_tG}@*.scalingVariable[time] := @{1}@.timeWorldToEnvModelConst;

--   @{rlc}@*.scalingVariable[distance] := rlc_v_delta * rlc_tB + rlc_v_delta * rlc_v_delta / (2 * rlc_a) + rlc_v_acsf * rlc_tG;
   @{rlc}@*.scalingVariable[distance] := (rlc_v_delta * 4) / 10 + rlc_v_delta * rlc_v_delta / (2 * rlc_a) + rlc_v_acsf * rlc_tG; -- Works with scaling = 1/1 only.

   rlc_lc_to_the_left_possible := (ego.gaps___609___.s_dist_rear > rlc) | (ego.gaps___609___.i_agent_rear = -(1)) | (rlc_v_acsf > rlc_v_rear);

   -- INIT
   -- (rlc < ego.gaps___609___.s_dist_rear) & (ego.gaps___609___.i_agent_rear != -(1)) & (rlc_v_rear > rlc_v_acsf);

}@********.if[@{FEASIBILITY}@.eval]
