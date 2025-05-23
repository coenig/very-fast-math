--------------------------------------------------------
--
-- Begin: Non-ego Spec (generate once per non-ego Vehicle)
--
--------------------------------------------------------


VAR
   cnt : integer;
   num_lanes : integer;

   @{ego.a : 0..0;
   ego.v : 0..0;}@**.if[@{EGOLESS}@.eval]

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




INVAR tar_dir = ActionDir____LEFT;
INIT cnt = 0;

@{
INIT 0 = @{section_[sec]_segment_0_pos_begin}@*.scalingVariable[distance];
INIT section_[sec]_segment_@{SEGMENTS - 1}@.eval[0]_pos_begin < section_[sec]_end;
@{
INIT section_[sec]_segment_[num]_pos_begin + @{SEGMENTSMINLENGTH}@.distanceWorldToEnvModelConst < section_[sec]_segment_@{[num] + 1}@.eval[0]_pos_begin;
INIT abs(section_[sec]_segment_[num]_min_lane - section_[sec]_segment_@{[num] + 1}@.eval[0]_min_lane) <= 1;
INIT abs(section_[sec]_segment_[num]_max_lane - section_[sec]_segment_@{[num] + 1}@.eval[0]_max_lane) <= 1;
}@**.for[[num], 0, @{SEGMENTS - 2}@.eval]

@{
INIT section_[sec]_segment_[num]_max_lane >= section_[sec]_segment_[num]_min_lane;
INIT section_[sec]_segment_[num]_min_lane >= 0;
INIT section_[sec]_segment_[num]_max_lane <= @{(NUMLANES - 1)}@.eval[0];
}@.for[[num], 0, @{SEGMENTS - 1}@.eval]

INIT section_[sec]_segment_0_min_lane = 0 & section_[sec]_segment_0_max_lane = @{(NUMLANES - 1)}@.eval[0]; -- Make sure we always have a drivable lane at the start. TODO: Make flexible.
}@***.for[[sec], 0, @{SECTIONS - 1}@.eval]

TRANS next(cnt) = cnt + 1;

DEFINE

@{ego.abs_pos := 0;}@**.if[@{EGOLESS}@.eval]

   large_number := 10000;

   @{
      section_[sec]_segment_@{SEGMENTS}@.eval[0]_pos_begin := section_[sec]_segment_@{SEGMENTS - 1}@.eval[0]_pos_begin + large_number; -- Helper variable to make below loop simpler.
   }@*.for[[sec], 0, @{SECTIONS - 1}@.eval]


   @{

   @{
   @{section_[sec]_lane_[lane]_availability_from_segment_@{SEGMENTS}@**.eval[0]}@*.scalingVariable[distance] := 0;          -- Helper variable to make below loop simpler.
   
   @{@{section_[sec]_lane_[lane]_availability_from_segment_[seg]}@*.scalingVariable[distance] := case
      section_[sec]_segment_[seg]_min_lane <= [lane] & section_[sec]_segment_[seg]_max_lane >= [lane] : section_[sec]_lane_[lane]_availability_from_segment_@{[seg] + 1}@.eval[0] + section_[sec]_segment_@{[seg] + 1}@.eval[0]_pos_begin - section_[sec]_segment_[seg]_pos_begin;
      TRUE: 0;
   esac;
   }@**.for[[seg], 0, @{SEGMENTS - 1}@.eval]
   }@****.for[[sec], 0, @{SECTIONS - 1}@.eval]

   @{ego.lane_[lane]_availability}@*.scalingVariable[distance] :=
   case 
   @{
      ego.on_section = [sec]:  
         case
            @{ego.abs_pos >= section_[sec]_segment_[seg]_pos_begin : section_[sec]_lane_[lane]_availability_from_segment_[seg] + section_[sec]_segment_[seg]_pos_begin - ego.abs_pos;
            }@.for[[seg], @{SEGMENTS - 1}@.eval, 0, -1]TRUE: 0;
         esac;
   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]
   esac;
   }@***.for[[lane], 0, @{NUMLANES - 1}@.eval]

 
@{
   @{veh___6[i]9___.abs_pos}@*.scalingVariable[distance] := veh___6[i]9___.rel_pos + ego.abs_pos;
}@**.for[[i], 0, @{NONEGOS - 1}@.eval]

@{

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

INVAR (ego.abs_pos < section_0_segment_0_pos_begin) -> 
   (ego.on_lane_min >= section_0_segment_0_min_lane & ego.on_lane_max <= section_0_segment_0_max_lane);

@{
   INVAR (ego.abs_pos >= section_0_segment_[num]_pos_begin & ego.abs_pos < section_0_segment_@{[num] + 1}@.eval[0]_pos_begin) -> 
   (ego.on_lane_min >= section_0_segment_[num]_min_lane & ego.on_lane_max <= section_0_segment_[num]_max_lane);
   }@*.for[[num], 0, @{SEGMENTS - 2}@.eval]
   INVAR (ego.abs_pos >= section_0_segment_@{SEGMENTS - 1}@.eval[0]_pos_begin) -> 
   (ego.on_lane_min >= section_0_segment_@{SEGMENTS - 1}@.eval[0]_min_lane & ego.on_lane_max <= section_0_segment_@{SEGMENTS - 1}@.eval[0]_max_lane);

}@**.if[@{!(EGOLESS)}@.eval]

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
   INVAR (veh___6[i]9___.abs_pos < section_0_segment_0_pos_begin) -> 
   (veh___6[i]9___.on_lane_min >= section_0_segment_0_min_lane & veh___6[i]9___.on_lane_max <= section_0_segment_0_max_lane);

   @{
   INVAR (veh___6[i]9___.abs_pos >= section_0_segment_[num]_pos_begin & veh___6[i]9___.abs_pos < section_0_segment_@{[num] + 1}@.eval[0]_pos_begin) -> 
   (veh___6[i]9___.on_lane_min >= section_0_segment_[num]_min_lane & veh___6[i]9___.on_lane_max <= section_0_segment_[num]_max_lane);
   }@*.for[[num], 0, @{SEGMENTS - 2}@.eval]

   INVAR (veh___6[i]9___.abs_pos >= section_0_segment_@{SEGMENTS - 1}@.eval[0]_pos_begin) -> 
   (veh___6[i]9___.on_lane_min >= section_0_segment_@{SEGMENTS - 1}@.eval[0]_min_lane & veh___6[i]9___.on_lane_max <= section_0_segment_@{SEGMENTS - 1}@.eval[0]_max_lane);

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

@{

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

}@**.if[@{!(EGOLESS)}@.eval]

   @{################# NOTE THAT THIS PART IS CHANGED AS COMPARED TO TACAS VERSION (might be inefficient) ###################}@.nil
   @{################# FORMERLY: veh___6[i]9___.same_lane_as_veh_[j] := ((veh___6[i]9___.lane_b1 & veh___6[j]9___.lane_b1) | (veh___6[i]9___.lane_b2 & veh___6[j]9___.lane_b2) | (veh___6[i]9___.lane_b3 & veh___6[j]9___.lane_b3)); ###################}@.nil
	@{
   veh___6[i]9___.same_lane_as_veh_[k] := (FALSE
      @{| ((veh___6[k]9___.lane_b[j] & veh___6[i]9___.lane_b[j]) @{) --}@**.if[@{EGOLESS}@.eval]  @{& !(veh___6[k]9___.lane_b@{[j]-1}@.eval[0] & veh___6[i]9___.lane_b@{[j]+1}@.eval[0]) & !(veh___6[k]9___.lane_b@{[j]+1}@.eval[0] & veh___6[i]9___.lane_b@{[j]-1}@.eval[0])}@.if[@{[j] > 0 && [j] < NUMLANES - 1}@.eval] )
      }@*.for[[j], 0, @{NUMLANES - 1}@.eval]
   );
   }@**.for[[k], 0, @{[i]}@.sub[1]]
	
	}@***.for[[i], 0, @{NONEGOS - 1}@.eval]


 @{

    crash := FALSE@{@{}@.space| ego.crash_with_veh_[i]}@*.for[[i], 0, @{NONEGOS - 1}@.eval];
    blamable_crash := FALSE@{@{}@.space| ego.blamable_crash_with_veh_[i]}@*.for[[i], 0, @{NONEGOS - 1}@.eval];
    

DEFINE -- TODO: Formerly INVAR, and "=" instead of ":=" - Check if this is correct!
ego.has_close_vehicle_on_left_left_lane := (FALSE @{@{ | ego.close_to_vehicle_[i]_on_left_left_lane}@.for[[i], 0, @{NONEGOS - 1}@.eval, 1]}@*.if[@{FARMERGINGCARS}@.eval]);     -- For some reason the "FALSE |"...
ego.has_close_vehicle_on_right_right_lane := (FALSE @{@{ | ego.close_to_vehicle_[i]_on_right_right_lane}@.for[[i], 0, @{NONEGOS - 1}@.eval, 1]}@*.if[@{FARMERGINGCARS}@.eval]); -- ...seems to be crucial for runtime.

}@**.if[@{!(EGOLESS)}@.eval]

INVAR
   num_lanes = @{NUMLANES}@.eval[0];

@{

-- Lookup table to speed-up non-linear calculations
DEFINE
square_of_ego_v := case
   @{ego.v = [x] : @{[x] ** 2}@.eval[0];
   }@*.for[[x], 0, @{@{MAXSPEEDEGO}@***.velocityWorldToEnvModelConst - 1}@.eval[0]]
   TRUE: @{@{MAXSPEEDEGO}@***.velocityWorldToEnvModelConst ** 2}@.eval[0];
esac;

}@**.if[@{!(EGOLESS)}@.eval]

@{
-- @{XVarEnvModelCarNote}@**.id
@{

INIT 
   veh___6[i]9___.rel_pos >= -@{INITPOSRANGENONEGOS}@.distanceWorldToEnvModelConst & veh___6[i]9___.rel_pos <= @{INITPOSRANGENONEGOS}@.distanceWorldToEnvModelConst;

-- Make sure ego and vehicle [i] don't collide in the initial state.
INIT
   veh___6[i]9___.rel_pos < veh_length | veh___6[i]9___.rel_pos > veh_length | !ego.same_lane_as_veh_[i];

}@**.if[@{!(EGOLESS)}@.eval]


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

@{

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

}@**.if[@{!(EGOLESS)}@.eval]

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
    @{init(veh___6[i]9___.a) := 0;}@**.if[@{!(EGOLESS)}@.eval]
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
            veh___6[i]9___.lc_direction = ActionDir____LEFT & !veh___6[i]9___.lane_max @{& !(ego.left_of_veh_[i]_lane & (next(veh___6[i]9___.rel_pos) > min_dist_long & next(veh___6[i]9___.rel_pos) < abs(min_dist_long) + veh_length))}@**.if[@{!(EGOLESS)}@.eval] : next(veh___6[i]9___.change_lane_now) = 0 ? veh___6[i]9___.lane_unchanged : veh___6[i]9___.lane_move_up;
            veh___6[i]9___.lc_direction = ActionDir____RIGHT & !veh___6[i]9___.lane_min @{& !(ego.right_of_veh_[i]_lane & (next(veh___6[i]9___.rel_pos) > min_dist_long & next(veh___6[i]9___.rel_pos) < abs(min_dist_long) + veh_length))}@**.if[@{!(EGOLESS)}@.eval] : next(veh___6[i]9___.change_lane_now) = 0 ? veh___6[i]9___.lane_unchanged : veh___6[i]9___.lane_move_down;
            TRUE : veh___6[i]9___.lane_unchanged;
        esac;
        -- at the latest point in time, we need to leave the source lane if we have not already
        veh___6[i]9___.lc_timer = leave_src_lane_latest_after & veh___6[i]9___.lane_single: 
        case
                veh___6[i]9___.lc_direction = ActionDir____LEFT & !veh___6[i]9___.lane_max @{& !(ego.left_of_veh_[i]_lane & (next(veh___6[i]9___.rel_pos) > min_dist_long & next(veh___6[i]9___.rel_pos) < abs(min_dist_long) + veh_length))}@**.if[@{!(EGOLESS)}@.eval] : veh___6[i]9___.lane_move_up; 
                veh___6[i]9___.lc_direction = ActionDir____RIGHT & !veh___6[i]9___.lane_min @{& !(ego.right_of_veh_[i]_lane & (next(veh___6[i]9___.rel_pos) > min_dist_long & next(veh___6[i]9___.rel_pos) < abs(min_dist_long) + veh_length))}@**.if[@{!(EGOLESS)}@.eval] : veh___6[i]9___.lane_move_down;
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
