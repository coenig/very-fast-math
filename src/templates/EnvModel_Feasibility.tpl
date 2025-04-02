DEFINE
   -- Rear-left Criticality (RLC)
   -- Caution: Works only with DISTANCESCALING = 10.
   @{rlc_v_rear}@*.scalingVariable[velocity] := ego.gaps___609___.v_rear;
   @{rlc_v_acsf}@*.scalingVariable[velocity] := ego.v;
   @{rlc_v_delta}@*.scalingVariable[velocity] := rlc_v_rear - rlc_v_acsf;
   @{rlc_tB}@*.scalingVariable[velocity] := @{0.4}@.velocityWorldToEnvModelConst;
   @{rlc_a}@*.scalingVariable[acceleration] := @{3}@.accelerationWorldToEnvModelConst;
   @{rlc_tG}@*.scalingVariable[velocity] := @{1}@.velocityWorldToEnvModelConst;
   @{rlc}@*.scalingVariable[distance] := rlc_v_delta * rlc_tB + rlc_v_delta * rlc_v_delta / (2 * rlc_a) + rlc_v_acsf * rlc_tG;
