--------------------------------------------------------
--
-- Begin: Section-graph reachability (REACHABILITY_ONLY mode)
--
-- Static transitive closure over the frozen outgoing-connection graph
-- (the "outgoing_connection_*_of_section_*" vars are FROZENVARs, so this
-- is a purely combinational Boolean function of the topology -- no driving
-- dynamics or temporal unrolling required). Source = section 0 (ego start),
-- target = section 1.
--
--------------------------------------------------------

DEFINE
   -- Direct successor relation: is section [sec2] a direct successor of section [sec]?
   @{
      @{
         @{
            is_direct_succ_[sec]_to_[sec2] := @{outgoing_connection_[con]_of_section_[sec] = [sec2]}@*.for[[con], 0, @{MAXOUTGOINGCONNECTIONS - 1}@.eval, 1, |];
         }@**.if[@{[sec] != [sec2]}@.eval]
      }@***.for[[sec], 0, @{SECTIONS - 1}@.eval]
   }@****.for[[sec2], 0, @{SECTIONS - 1}@.eval]

   -- Reachability layer 0: only the source section (0) is reached.
   @{
      reach_0_of_sec_[sec] := @{@(TRUE)@@(FALSE)@}@*.if[@{[sec] == 0}@.eval];
   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]

   -- Reachability layers 1..SECTIONS-1: one Bellman-Ford/BFS relaxation each.
   -- reach_{k+1}(sec) = reach_k(sec) | OR_m (reach_k(m) & is_direct_succ(m, sec)).
   @{
      @{
         reach_@{[k] + 1}@.eval[0]_of_sec_[sec] := reach_[k]_of_sec_[sec]@{ @{| (reach_[k]_of_sec_[m] & is_direct_succ_[m]_to_[sec])}@*.if[@{[m] != [sec]}@.eval]}@**.for[[m], 0, @{SECTIONS - 1}@.eval];
      }@***.for[[sec], 0, @{SECTIONS - 1}@.eval]
   }@****.for[[k], 0, @{SECTIONS - 2}@.eval]

   -- Is the target section (1) reachable from the source section (0)?
   is_target_reachable := reach_@{SECTIONS - 1}@.eval[0]_of_sec_1;

--------------------------------------------------------
-- EO Section-graph reachability
--------------------------------------------------------
