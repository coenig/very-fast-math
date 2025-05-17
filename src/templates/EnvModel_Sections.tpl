
--------------------------------------------------------
-- Sections
--------------------------------------------------------

   -- INIT outgoing_connection_0_of_section_0 != -1; -- Make first section split...
   -- INIT outgoing_connection_1_of_section_0 != -1; -- ...into two target roads.

   @{
      FROZENVAR
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

         @{
         INVAR 
            @{ outgoing_connection_0_of_section_[sec2] = [sec] }@.for[[sec2], 0, @{[sec]}@.sub[1], 1, |];
         }@.if[@{[sec] > 0}@.eval]

      @{
         INIT @{ vec(section_[sec].source.x; section_[sec].source.y) }@.syntacticMaxCoordDistance[ vec(section_[sec2].source.x; section_[sec2].source.y) ] 
            >= @{MINDISTCONNECTIONS}@.eval[0]; -- Use only farther away coordinate as lower bound.
         INIT @{ vec(section_[sec].source.x; section_[sec].source.y) }@.syntacticMaxCoordDistance[ vec(section_[sec2].drain.x; section_[sec2].drain.y) ] 
            >= @{MINDISTCONNECTIONS}@.eval[0]; -- Use only farther away coordinate as lower bound.
         INIT @{ vec(section_[sec].drain.x; section_[sec].drain.y) }@.syntacticMaxCoordDistance[ vec(section_[sec2].source.x; section_[sec2].source.y) ] 
            >= @{MINDISTCONNECTIONS}@.eval[0]; -- Use only farther away coordinate as lower bound.
      }@*.for[[sec2], 0, @{[sec]}@.sub[1]]

   }@***.for[[sec], 0, @{SECTIONS - 1}@.eval]

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
