@{EnvModel_Common_Sections.tpl}@********.include
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
INIT section_[sec]_segment_@{SEGMENTS - 1}@.eval[0]_pos_begin < section_[sec]_end | section_[sec]_segment_@{SEGMENTS - 1}@.eval[0]_pos_begin = 0;
@{
INIT section_[sec]_segment_[num]_pos_begin + @{SEGMENTSMINLENGTH}@.distanceWorldToEnvModelConst < section_[sec]_segment_@{[num] + 1}@.eval[0]_pos_begin;
INIT abs(section_[sec]_segment_[num]_min_lane - section_[sec]_segment_@{[num] + 1}@.eval[0]_min_lane) <= 1;
INIT abs(section_[sec]_segment_[num]_max_lane - section_[sec]_segment_@{[num] + 1}@.eval[0]_max_lane) <= 1;
}@**.for[[num], 0, @{SEGMENTS - 2}@.eval]
}@**.for[[sec], 0, @{SECTIONS - 1}@.eval]

--------------------------------------------------------
--  <== EO Segments
--------------------------------------------------------


   @{
      FROZENVAR
         @{
            @{section_[sec]_segment_[seg]_pos_begin}@*.scalingVariable[distance] : integer;
         }@**.for[[seg], 0, @{SEGMENTS - 1}@.eval]
   
         @{section_[sec]_end}@*.scalingVariable[distance] : 
            @{@(0)@@(@{SECTIONSMINLENGTH}@.eval[0])@}@******.if[@{ALLOW_ZEROLENGTH_SECTIONS}@.eval] .. @{SECTIONSMAXLENGTH}@.eval[0]; -- This is essentially the length of the section.

         section_[sec].source.x : integer;
         section_[sec].source.y : integer;
         section_[sec].angle_raw : 0 .. @{ trunc(359 / ANGLEGRANULARITY) }@.eval[0];

      @{
      INIT section_[sec]_end = 0                               -- Allow straight sections with zero length.
         | section_[sec]_end >= @{SECTIONSMINLENGTH}@.eval[0];
      }@******.if[@{ALLOW_ZEROLENGTH_SECTIONS}@.eval]

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
 
      @{
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
