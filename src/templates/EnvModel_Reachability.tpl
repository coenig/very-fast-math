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

-- The section topology (outgoing_connection_*_of_section_*) plus section geometry and the
-- rect_obstacles_* DEFINEs / obstacle-avoidance INIT constraints are provided by
-- EnvModel_Sections.tpl (included in reachability mode too). Here we only add the
-- transitive-closure reachability DEFINEs on top of that frozen topology.

DEFINE
   -- Direct successor relation: is section [sec2] a direct successor of section [sec]?
   -- (i.e. driving FORWARD out of [sec]'s drain leads into [sec2]'s source).
   @{
      @{
         @{
            is_direct_succ_[sec]_to_[sec2] := @{outgoing_connection_[con]_of_section_[sec] = [sec2]}@*.for[[con], 0, @{MAXOUTGOINGCONNECTIONS - 1}@.eval, 1, |];
         }@**.if[@{[sec] != [sec2]}@.eval]
      }@***.for[[sec], 0, @{SECTIONS - 1}@.eval]
   }@****.for[[sec2], 0, @{SECTIONS - 1}@.eval]

   -- Forward-terminal: no outgoing connection at all (a forward dead-end pocket). The ego can
   -- drive in nose-first and only continue by reversing gear (F -> B) there.
   @{
      is_forward_terminal_[sec] := @{outgoing_connection_[con]_of_section_[sec] = -1}@*.for[[con], 0, @{MAXOUTGOINGCONNECTIONS - 1}@.eval, 1, &];
   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]

   -- Backward-terminal: no INCOMING connection at all (a backward dead-end pocket). The mirror of
   -- forward-terminal: the ego arrives in reverse (using one of the section's own outgoing
   -- connections as a backward-incoming route), dead-ends at the source, and flips gear (B -> F).
   @{
      is_backward_terminal_[sec] := TRUE@{ @{& !is_direct_succ_[m]_to_[sec]}@*.if[@{[m] != [sec]}@.eval]}@**.for[[m], 0, @{SECTIONS - 1}@.eval];
   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]

   -- Oriented reachability over 2*SECTIONS nodes: (section, direction) with direction
   -- F = moving from source towards drain, B = moving from drain towards source.
   -- Layer 0 seeds: at the source section (0) the ego may start parking out either
   -- forward or backward, so both orientations are reachable there.
   @{
      reach_0_of_sec_[sec]_dir_F := @{@(TRUE)@@(FALSE)@}@*.if[@{[sec] == 0}@.eval];
      reach_0_of_sec_[sec]_dir_B := @{@(TRUE)@@(FALSE)@}@*.if[@{[sec] == 0}@.eval];
   }@**.for[[sec], 0, @{SECTIONS - 1}@.eval]

   -- Layers 1..2*SECTIONS-1: one BFS relaxation each over the oriented edges:
   --   forward chain  (m,F) -> (sec,F)  iff is_direct_succ(m, sec)   (m -> sec)
   --   backward chain (m,B) -> (sec,B)  iff is_direct_succ(sec, m)   (sec -> m, traversed in reverse)
   --   reversal F->B  (sec,F) -> (sec,B) iff is_forward_terminal(sec)  (nose into a dead end, back out)
   --   reversal B->F  (sec,B) -> (sec,F) iff is_backward_terminal(sec) (reverse into a pocket, drive out)
   -- Reversal is forbidden AT the target (sec 1) so its park-in orientation stays distinct
   -- (forward = nose-in via an incoming connection, backward = tail-in via an outgoing one).
   @{
      @{
         reach_@{[k] + 1}@.eval[0]_of_sec_[sec]_dir_F := reach_[k]_of_sec_[sec]_dir_F@{ @{| (reach_[k]_of_sec_[m]_dir_F & is_direct_succ_[m]_to_[sec])}@*.if[@{[m] != [sec]}@.eval]}@**.for[[m], 0, @{SECTIONS - 1}@.eval]@{ | (reach_[k]_of_sec_[sec]_dir_B & is_backward_terminal_[sec])}@.if[@{[sec] != 1}@.eval];
         reach_@{[k] + 1}@.eval[0]_of_sec_[sec]_dir_B := reach_[k]_of_sec_[sec]_dir_B@{ @{| (reach_[k]_of_sec_[m]_dir_B & is_direct_succ_[sec]_to_[m])}@*.if[@{[m] != [sec]}@.eval]}@**.for[[m], 0, @{SECTIONS - 1}@.eval]@{ | (reach_[k]_of_sec_[sec]_dir_F & is_forward_terminal_[sec])}@.if[@{[sec] != 1}@.eval];
      }@***.for[[sec], 0, @{SECTIONS - 1}@.eval]
   }@****.for[[k], 0, @{2 * SECTIONS - 2}@.eval]

   -- Is the target section (1) reachable, entering it forward / backward?
   is_target_reachable_forward  := reach_@{2 * SECTIONS - 1}@.eval[0]_of_sec_1_dir_F;
   is_target_reachable_backward := reach_@{2 * SECTIONS - 1}@.eval[0]_of_sec_1_dir_B;

   -- Overall target predicate, selected by the required park-in direction(s):
   --   PARK_IN_FORWARD / PARK_IN_BACKWARD (both true = either direction accepted).
   is_target_reachable := FALSE@{ | is_target_reachable_forward}@.if[@{PARK_IN_FORWARD}@.eval]@{ | is_target_reachable_backward}@.if[@{PARK_IN_BACKWARD}@.eval];

--------------------------------------------------------
-- EO Section-graph reachability
--------------------------------------------------------
