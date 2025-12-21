
--------------------------------------------------------
-- Sections
--  ==> Segments
--------------------------------------------------------

@{
@{
INIT section_[sec]_segment_[num]_max_lane >= section_[sec]_segment_[num]_min_lane;
}@.for[[num], 0, @{SEGMENTS - 1}@.eval]
}@***.for[[sec], 0, @{SECTIONS - 1}@.eval]


--------------------------------------------------------
--  <== EO Segments
--------------------------------------------------------


   @{
      FROZENVAR
         @{
            section_[sec]_segment_[seg]_min_lane : 0 .. @{NUMLANES - 1}@.eval[0];
            section_[sec]_segment_[seg]_max_lane : 0 .. @{NUMLANES - 1}@.eval[0];
            @{section_[sec]_segment_[seg]_pos_begin}@*.scalingVariable[distance] : @{@(integer)@@(0 .. 1)@}@.if[@{CONCRETE_MODEL}@.eval];
         }@**.for[[seg], 0, @{SEGMENTS - 1}@.eval]
   
      @{
      FROZENVAR outgoing_connection_[con]_of_section_[sec] : -1..@{SECTIONS - 1}@.eval[0];
      INIT outgoing_connection_[con]_of_section_[sec] != [sec]; -- Do not connect to self.
      @{
      INIT outgoing_connection_[con]_of_section_[sec] = -1 -> outgoing_connection_@{[con] + 1}@.eval[0]_of_section_[sec] = -1;
      }@***.if[@{[con] < MAXOUTGOINGCONNECTIONS -1}@.eval] -- Reduce topological permutations
      }@****.for[[con], 0, @{MAXOUTGOINGCONNECTIONS-1}@.eval] -- Several elements can be equal, so we have at least 1 and at most @{MAXOUTGOINGCONNECTIONS}@.eval[0] outgoing connections.
   }@******.for[[sec], 0, @{SECTIONS - 1}@.eval]

--------------------------------------------------------
-- EO Sections
--------------------------------------------------------

VAR
   num_lanes : 0 .. @{NUMLANES}@.eval[0];
INVAR num_lanes = @{NUMLANES}@.eval[0];
