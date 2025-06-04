
--------------------------------------------------------
-- Sections
--  ==> Segments
--------------------------------------------------------

   DEFINE
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

--------------------------------------------------------
--  <== EO Segments
--------------------------------------------------------

   -- TODO: Needs to be removed again
   -- CROSSING
   -- INIT outgoing_connection_0_of_section_0 = 1;
   -- INIT outgoing_connection_1_of_section_0 = 3;
   -- INIT outgoing_connection_2_of_section_0 = 5;
   -- INIT outgoing_connection_0_of_section_2 = 3;
   -- INIT outgoing_connection_1_of_section_2 = 5;
   -- INIT outgoing_connection_2_of_section_2 = 7;
   -- INIT outgoing_connection_0_of_section_4 = 1;
   -- INIT outgoing_connection_1_of_section_4 = 7;
   -- INIT outgoing_connection_2_of_section_4 = 5;
   -- INIT outgoing_connection_0_of_section_6 = 7;
   -- INIT outgoing_connection_1_of_section_6 = 1;
   -- INIT outgoing_connection_2_of_section_6 = 3;
   -- INIT section_0.angle = 0;
   -- INIT section_1.angle = 90;
   -- INIT section_2.angle = 270;
   -- INIT section_3.angle = 0;
   -- INIT section_4.angle = 180;
   -- INIT section_5.angle = 270;
   -- INIT section_6.angle = 90;
   -- INIT section_7.angle = 180;
   -- INIT section_4.source.y <= section_0.source.y - 6;
   -- INIT section_1.source.x <= section_2.source.x - 6;
   -- INIT veh___609___.is_on_sec_4 = 1;
   -- INIT veh___609___.abs_pos = 0;
   -- INIT veh___619___.is_on_sec_6 = 1;
   -- INIT veh___619___.abs_pos = 0;
   -- INIT veh___629___.is_on_sec_0 = 1;
   -- INIT veh___629___.abs_pos = 0;
   -- INIT veh___639___.is_on_sec_2 = 1;
   -- INIT veh___639___.abs_pos = 0;
   -- INIT section_0.drain.x = section_7.source.x;

   -- ROUNDABOUT
   INIT section_0.angle = 0;
   INIT section_1.angle = 90;
   INIT section_2.angle = 270;
   INIT section_3.angle = 0;
   INIT section_4.angle = 180;
   INIT section_5.angle = 270;
   INIT section_6.angle = 90;
   INIT section_7.angle = 180;
   INIT section_8.angle = 45;
   INIT section_9.angle = 315;
   INIT section_10.angle = 225;
   INIT section_11.angle = 135;

   INIT outgoing_connection_0_of_section_0 = 8;
   INIT outgoing_connection_0_of_section_1 = -1;
   INIT outgoing_connection_0_of_section_2 = 9;
   INIT outgoing_connection_0_of_section_3 = -1;
   INIT outgoing_connection_0_of_section_4 = 10;
   INIT outgoing_connection_0_of_section_5 = -1;
   INIT outgoing_connection_0_of_section_6 = 11;
   INIT outgoing_connection_0_of_section_7 = -1;
   INIT outgoing_connection_0_of_section_8 = 1;
   INIT outgoing_connection_0_of_section_9 = 3;
   INIT outgoing_connection_0_of_section_10 = 5;
   INIT outgoing_connection_0_of_section_11 = 7;
   INIT outgoing_connection_1_of_section_8 = 9;
   INIT outgoing_connection_1_of_section_9 = 10;
   INIT outgoing_connection_1_of_section_10 = 11;
   INIT outgoing_connection_1_of_section_11 = 8;
   -- EO TODO: Needs to be removed again

   @{
      FROZENVAR
         @{
            @{section_[sec]_segment_[seg]_pos_begin}@*.scalingVariable[distance] : integer;
            section_[sec]_segment_[seg]_min_lane : integer;
            section_[sec]_segment_[seg]_max_lane : integer;
         }@**.for[[seg], 0, @{SEGMENTS - 1}@.eval]
   
         @{section_[sec]_end}@*.scalingVariable[distance] : @{SECTIONSMINLENGTH}@.eval[0] .. @{SECTIONSMAXLENGTH}@.eval[0];

         section_[sec].source.x : integer;
         section_[sec].source.y : integer;
         section_[sec].angle_raw : 0 .. @{ trunc(359 / ANGLEGRANULARITY) }@.eval[0];

      DEFINE 
         section_[sec].angle := section_[sec].angle_raw * @{ANGLEGRANULARITY}@.eval[0];

      -- Lookup table to speed-up non-linear calculations (sin times 100)
      sin_of_section_[sec]_angle := case
         @{
            section_[sec].angle = [x] : @{ sin([x] / 360 * 2 * 3.1415) * 100 }@.eval[0];
         }@*.for[[x], 0, 359, @{ANGLEGRANULARITY}@.eval]
      esac;

      -- Lookup table to speed-up non-linear calculations (cos times 100)
      cos_of_section_[sec]_angle := case
         @{
            section_[sec].angle = [x] : @{ cos([x] / 360 * 2 * 3.1415) * 100 }@.eval[0];
         }@*.for[[x], 0, 359, @{ANGLEGRANULARITY}@.eval]
      esac;

      section_[sec].drain.x := section_[sec].source.x + (section_[sec]_end * cos_of_section_[sec]_angle) / 100;
      section_[sec].drain.y := section_[sec].source.y + (section_[sec]_end * sin_of_section_[sec]_angle) / 100;
 
      -- INIT outgoing_connection_0_of_section_[sec] != outgoing_connection_1_of_section_[sec]; -- If we want to have at least one outgoing connection for all roads.

         @{
            FROZENVAR outgoing_connection_[con]_of_section_[sec] : -1..@{SECTIONS - 1}@.eval[0];
            INIT outgoing_connection_[con]_of_section_[sec] != [sec]; -- Don't connect to self.

            @{
               @{
                  FROZENVAR dist_[con]_of_section_[sec]_to_[sec2] : @{MINDISTCONNECTIONS}@.eval[0] .. @{MAXDISTCONNECTIONS}@.eval[0];

                  INIT outgoing_connection_[con]_of_section_[sec] = [sec2] -> (
                       (section_[sec2].source.x = section_[sec].drain.x + (dist_[con]_of_section_[sec]_to_[sec2] * (cos_of_section_[sec]_angle + cos_of_section_[sec2]_angle)) / 100)
                     & (section_[sec2].source.y = section_[sec].drain.y + (dist_[con]_of_section_[sec]_to_[sec2] * (sin_of_section_[sec]_angle + sin_of_section_[sec2]_angle)) / 100)
                  );
               }@.if[@{ [sec] != [sec2] }@.eval]
            }@*.for[[sec2], 0, @{SECTIONS - 1}@.eval]
         }@**.for[[con], 0, @{MAXOUTGOINGCONNECTIONS-1}@.eval] -- Several elements can be equal, so we have at least 1 and at most @{MAXOUTGOINGCONNECTIONS}@.eval[0] outgoing connections.

         @{ -- TODO: Comment in to make all sections connect.
         -- INVAR 
            @{ -- outgoing_connection_0_of_section_[sec2] = [sec] }@.for[[sec2], 0, @{[sec]}@.sub[1], 1, |];
         }@.if[@{[sec] > 0}@.eval]

      @{
         INIT @{ vec(section_[sec].source.x; section_[sec].source.y) }@.syntacticMaxCoordDistance[ vec(section_[sec2].source.x; section_[sec2].source.y) ] 
            >= @{MAXDISTENDPOINTS}@.eval[0]; -- Use only farther away coordinate as lower bound.
         INIT @{ vec(section_[sec].source.x; section_[sec].source.y) }@.syntacticMaxCoordDistance[ vec(section_[sec2].drain.x; section_[sec2].drain.y) ] 
            >= @{MAXDISTENDPOINTS}@.eval[0]; -- Use only farther away coordinate as lower bound.
         INIT @{ vec(section_[sec].drain.x; section_[sec].drain.y) }@.syntacticMaxCoordDistance[ vec(section_[sec2].source.x; section_[sec2].source.y) ] 
            >= @{MAXDISTENDPOINTS}@.eval[0]; -- Use only farther away coordinate as lower bound.
      }@*.for[[sec2], 0, @{[sec]}@.sub[1]]

      @{
         @{
         DEFINE
            angle_from_sec_[sec]_to_sec_[sec2]_raw := section_[sec2].angle - section_[sec].angle;
            angle_from_sec_[sec]_to_sec_[sec2] := case
               angle_from_sec_[sec]_to_sec_[sec2]_raw < 0 : angle_from_sec_[sec]_to_sec_[sec2]_raw + 360;
               angle_from_sec_[sec]_to_sec_[sec2]_raw >= 360 : angle_from_sec_[sec]_to_sec_[sec2]_raw - 360;
               TRUE : angle_from_sec_[sec]_to_sec_[sec2]_raw;
            esac;
            connection_distance_sec_[sec]_to_sec_[sec2] := case -- TODO: We could have several connections for the same sections with different conn. distances.
               @{
                  outgoing_connection_[con]_of_section_[sec] = [sec2] : dist_[con]_of_section_[sec]_to_[sec2];
               }@*.for[[con], 0, @{MAXOUTGOINGCONNECTIONS-1}@.eval]
               TRUE : -1;
            esac;

            @{
               @{arclength_from_sec_[sec]_to_sec_[sec2]_on_lane_[lane]}@*.scalingVariable[distance] := case
                  @{@{
                           angle_from_sec_[sec]_to_sec_[sec2] = [angle] & connection_distance_sec_[sec]_to_sec_[sec2] = [dist] : @{@{[lane]}@.arclengthCubicBezierFromStreetTopology[[angle], [dist], @{NUMLANES}@.eval[0]]}@.distanceWorldToEnvModelConst;
                     }@*.for[[dist], @{MINDISTCONNECTIONS}@.eval, @{MAXDISTCONNECTIONS}@.eval]
                  }@**.for[[angle], 0, 359, @{ANGLEGRANULARITY}@.eval]
                  TRUE : -1;
               esac;
            }@***.for[[lane], 0, @{NUMLANES - 1}@.eval]
         }@****.if[@{[sec] != [sec2]}@.eval]
      }@*****.for[[sec2], 0, @{SECTIONS - 1}@.eval]
   }@******.for[[sec], 0, @{SECTIONS - 1}@.eval]

   -- Section 0 always starts at (0/0) and goes horizontally to the right.
   INIT section_0.source.x = 0;
   INIT section_0.source.y = 0;
   -- INIT section_0.drain.x ==> Not specified, so the length of the section is figured out from the length of the segments.
   INIT section_0.angle = 0;

VAR    
   ego.on_section : 0 .. @{SECTIONS - 1}@.eval[0];

INIT ego.on_section = 0;

--------------------------------------------------------
-- EO Sections
--------------------------------------------------------
