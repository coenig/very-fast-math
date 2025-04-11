
--------------------------------------------------------
-- Sections
--------------------------------------------------------

   @{
      VAR
         section___6[sec]9___.source.x : @{BORDERLEFT}@.eval[0] .. @{BORDERRIGHT}@.eval[0];
         section___6[sec]9___.source.y : @{BORDERTOP}@.eval[0] .. @{BORDERBOTTOM}@.eval[0];
         section___6[sec]9___.drain.x : @{BORDERLEFT}@.eval[0] .. @{BORDERRIGHT}@.eval[0];
         section___6[sec]9___.drain.y : @{BORDERTOP}@.eval[0] .. @{BORDERBOTTOM}@.eval[0];

      ASSIGN
         next(section___6[sec]9___.source.x) := section___6[sec]9___.source.x;
         next(section___6[sec]9___.source.y) := section___6[sec]9___.source.y;
         next(section___6[sec]9___.drain.x) := section___6[sec]9___.drain.x;
         next(section___6[sec]9___.drain.y) := section___6[sec]9___.drain.y;

      INIT section___6[sec]9___.source.x mod @{COORDGRANULARITY}@.eval[0] = 0;
      INIT section___6[sec]9___.source.y mod @{COORDGRANULARITY}@.eval[0] = 0;
      INIT section___6[sec]9___.drain.x mod @{COORDGRANULARITY}@.eval[0] = 0;
      INIT section___6[sec]9___.drain.y mod @{COORDGRANULARITY}@.eval[0] = 0;

      @{
        -- @{ vec(section___6[sec]9___.source.x; section___6[sec]9___.source.y) }@.syntacticSquareOfVecDistance[ vec(section___6[sec2]9___.source.x; section___6[sec2]9___.source.y) ] >= 
      }@*.for[[sec2], 0, @{[sec]}@.sub[1]]

   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]

--------------------------------------------------------
-- EO Sections
--------------------------------------------------------
