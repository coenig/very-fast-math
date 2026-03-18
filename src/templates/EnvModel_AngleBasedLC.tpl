@{
-- Angle-based lanechange


DEFINE
   @{max_lat_acceleration}@*.accelerationWorldToEnvModelDef[MAX_LAT_ACCEL];
   hundret_pi := 314;
  

   -- Minimum and maximum velocities
   @{
      @{@OVERALL_V_MAX = -1000000}@******.eval
      @{@OVERALL_V_MIN = 1000000}@******.eval

      @{
         @{@OVERALL_V_MAX = max(OVERALL_V_MAX, @{[el]}@**.timeWorldToEnvModelConst)}@**.eval
         @{@OVERALL_V_MIN = min(OVERALL_V_MIN, @{[el]}@**.timeWorldToEnvModelConst)}@**.eval
      }@***.for[[el], @{LANES_MIN_SPEEDS}@****.printHeap@{LANES_MAX_SPEEDS}@****.printHeap]
   }@.nil

      overall_max_v := @{OVERALL_V_MAX}@.eval[0];
      overall_min_v := @{OVERALL_V_MIN}@.eval[0];
   -- EO Minimum and maximum velocities
@{
DEFINE

@{ @abs_range = 0.5 }@***.eval.nil

@{
   veh___6[i]9___.lc_angle_max_deg := case
   @{
   @{
       @{max(min(LANE_WIDTH * 3.141592 / ([v] * [T]), abs_range), -abs_range)}@.setScriptVar[temp, force].nil
       
       veh___6[i]9___.v = [v] & veh___6[i]9___.current_lc_duration = [T] : @{180 / 3.141592 * arcsin(@{temp}@.scriptVar)}@.eval[0];
   }@*.for[[T], @{MIN_LC_DURATION_SEC}@**.timeWorldToEnvModelConst, @{MAX_LC_DURATION_SEC}@**.timeWorldToEnvModelConst]
   }@**.for[[v], @{OVERALL_V_MIN}@.eval, @{OVERALL_V_MAX}@.eval]
       TRUE : 0;
   esac;
}@.removeBlankLines

VAR
   veh___6[i]9___.current_lc_duration : @{MIN_LC_DURATION_SEC}@.timeWorldToEnvModelConst .. @{MAX_LC_DURATION_SEC}@.timeWorldToEnvModelConst;
   veh___6[i]9___.current_angle : integer;
   veh___6[i]9___.lateral_offset : 0 .. @{LATERAL_STEPS_BETWEEN_LANES - 1}@.eval[0];
}@***.for[[i], 0, @{NONEGOS - 1}@.eval]



-- EO Angle-based lanechange
}@******.if[@{ANGLE_BASED_LC}@.eval]