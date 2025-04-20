
--------------------------------------------------------
-- Sections
--------------------------------------------------------

   @{
      FROZENVAR
         section_[sec].source.x_raw : @{BORDERLEFT / COORDGRANULARITY}@.eval[0] .. @{BORDERRIGHT / COORDGRANULARITY}@.eval[0];
         section_[sec].source.y_raw : @{BORDERTOP / COORDGRANULARITY}@.eval[0] .. @{BORDERBOTTOM / COORDGRANULARITY}@.eval[0];
         section_[sec].angle : 0 .. 359;
      
      DEFINE 
      section_[sec].source.x := section_[sec].source.x_raw * @{COORDGRANULARITY}@.eval[0];
      section_[sec].source.y := section_[sec].source.y_raw * @{COORDGRANULARITY}@.eval[0];

         @{
            VAR outgoing_connection_[con]_of_section_[sec] : 0..@{SECTIONS - 1}@.eval[0];
            INVAR outgoing_connection_[con]_of_section_[sec] != [sec]; -- Don't connect to self.
         }@.for[[con], 0, @{MAXOUTGOINGCONNECTIONS-1}@.eval] -- Several elements can be equal, so we have at least 1 and at most @{MAXOUTGOINGCONNECTIONS}@.eval[0] outgoing connections.

      @{
        INIT @{ vec(section_[sec].source.x; section_[sec].source.y) }@.syntacticManhattenDistance[ vec(section_[sec2].source.x; section_[sec2].source.y) ] 
            <= @{MAXDISTCONNECTIONS}@.eval[0]; -- Use Manhatten distance as upper bound.
        INIT @{ vec(section_[sec].source.x; section_[sec].source.y) }@.syntacticMaxCoordDistance[ vec(section_[sec2].source.x; section_[sec2].source.y) ] 
            >= @{MINDISTCONNECTIONS}@.eval[0]; -- Use only farther away coordinate as lower bound.
      }@*.for[[sec2], 0, @{[sec]}@.sub[1]]

   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]

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
