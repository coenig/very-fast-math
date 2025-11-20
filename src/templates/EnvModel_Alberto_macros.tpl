-- TODO[AB]: I will have to change the name to this field!!!
@{#0#.on_secpart}@**.newMethod[secpart, 0]

@{#0#.traversing_to}@**.newMethod[to, 0]

@{#0#.traversing_from}@**.newMethod[from, 0]

@{#0#.on_straight_section}@**.newMethod[sec, 0]

@{outgoing_connection_#1#_of_section_#0#}@**.newMethod[outconn, 1]

@{section_#0#_n_secparts}@**.newMethod[nsecparts, 0]

-- Expression is true if the two cars are on the same section.
--
--#define samesec(c0, c1) \
--  (c0.on_straight_section != -1 & c0.on_straight_section = c1.on_straight_section)
@{ (@{#0#}@.sec = @{#1#}@.sec & @{#0#}@.sec != -1) }@*.newMethod[samesec, 1]

-- True if car0 is in the same section of car1 and it is behind.
-- If the two cars are in different sections or in junctions the proposition is false
--
--#define behindsec(c0, c1) \
--  (samesec(c0, c1) & (c0.on_secpart < c1.on_secpart))
@{
  (@{#0#}@.samesec[#1#] & @{#0#}@.secpart < @{#0#}@.secpart)
}@**.newMethod[behindsec, 1]

-- True if the car is in the last chunk of the section (\sa on_secpart)
--
--#define car_at_end_of_section(c0, sec_id, sec_n_seg) \
--  (c0.on_straight_section = sec_id & c0.on_secpart = sec_n_seg)
--
-- TODO[AB]: Extend with new MAX_SECPART parameter?
--@{ @{#0#}@.sec = #1# & @{#0#}@.secpart = #2# }@**.newMethod[carendsect, 2]


@{
  VAR
    @{#0#}@.secpart : -1 .. #2#;
    @{#0#}@.sec : -1 .. #1#;
    @{#0#}@.from : -1 .. #1#;
    @{#0#}@.to : -1 .. #1#;
  INIT @{#0#}@.to = -1 & @{#0#}@.from = -1;
  INIT @{#0#}@.secpart != -1;
  TRANS @{#0#}@.sec = -1 ->
    (next(@{#0#}@.sec) = -1 & next(@{#0#}@.to) = @{#0#}@.to & next(@{#0#}@.from) = @{#0#}@.from & next(@{#0#}@.secpart = -1)) |
    (next(@{#0#}@.sec) = @{#0#}@.to & next(@{#0#}@.to) = -1 & next(@{#0#}@.from) = -1 & next(@{#0#}@.secpart) = 0);
}@***.newMethod[CarCoreDecl, 2]

-- Constraints to move from the current section part to the next one. We ensure that we are in the range with n_secparts variable.
--
--#define _do_progress_secpart(c0, sec0) \
--  (c0.on_secpart < sec0.n_secparts & next(c0.on_secpart) = c0.on_secpart + 1)
--
@{
  (@{#0#}@.secpart < @{#1#}@.nsecparts & next(@{#0#}@.secpart) = @{#0#}@.secpart + 1)
}@***.newMethod[doprogresssecpart,1]

-- True if it is possible to pass from a section to the next one (assuming that c0.on_straight_section = sec0_id)
--
--#define cantraverse(c0, sec0) \
--  (c0.on_secpart = sec0.n_secparts)
--#endif
--
@{
  (@{#0#}@.secpart = @{#1#}@.nsecparts)
}@***.newMethod[cantraverse, 1]

-- Behavior of car while moving inside the section
--#define CarTransSectInner(c0, sec0, sec0_id) \
--  INVAR (c0.on_straight_section = sec0_id) -> c0.on_secpart >= 0 & c0.on_secpart <= sec0.n_secparts; \
--  TRANS (c0.on_straight_section = sec0_id) & next(c0.on_straight_section) != -1 ->  \
--    (next(c0.on_straight_section = sec0_id) & (next(c0.on_secpart) = c0.on_secpart | _do_progress_secpart(c0, sec0)) & \
--     next(c0.on_secpart) <= sec0.n_secparts & \
--     next(c0.traversing_to) = -1 & next(c0.traversing_from) = -1)
--
@{
  INVAR (@{#0#}@.sec = #1#) -> @{#0#}@.secpart >= 0 & @{#0#}@.secpart <= @{#1#}@.nsecparts; 
  TRANS (@{#0#}@.sec = #1#) & next(@{#0#}@.sec) != -1 ->  
    (next(@{#0#}@.sec = #1#) & (next(@{#0#}@.secpart) = @{#0#}@.secpart | @{#0#}@.doprogresssecpart[#1#] & 
     next(@{#0#}@.secpart) <= @{#1#}@.nsecparts & 
     next(@{#0#}@.to) = -1 & next(@{#0#}@.from) = -1))
}@***.newMethod[CarTransSectInner, 1]

-- True if it will pass from a section to a junction
--#define willtraverse(c0, sec0_id) \
--  (c0.on_straight_section = sec0_id & next(c0.on_straight_section) = -1)
--
@{@{#0#}@.sec = #1# & next(@{#0#}@.sec) = -1}@***.newMethod[willtraverse, 1]

-- Model the transitions between a section and a junction
--
--#define CarTransSectJunc(c0, sec0, sec0_id, junc_big_or) \
--  TRANS willtraverse(c0, sec0_id) -> (cantraverse(c0, sec0) & next(c0.on_secpart) = -1 & next(c0.traversing_from) = sec0_id & next(c0.traversing_to) != -1 & (junc_big_or))
--
@{
  TRANS @{#0#}@.willtraverse[#1#] -> (@{#0#}@.cantraverse[#1#] & next(@{#0#}@.secpart) = -1 & next(@{#0#}@.from) = #1# & next(@{#0#}@.to) != -1 &
    (@{
      @{#1#}@.outconn[[con]] = next(@{#0#}@.to)
    }@*.for[[con],0, @{MAXOUTGOINGCONNECTIONS - 1}@.eval[0], 1,|])
    )
}@***.newMethod[CarTransSectJunc, 1]

-- Conditions such that a car is considered behind another car. Uses an additional variable that looks at the past to know if it is still behind.
--
--#define behindconds(c0, c1, was_behind_var) \
--  (((behindsec(c0, c1)) | \
--        (was_behind_var & (c0.on_straight_section = c1.traversing_from | \
--                           c0.on_straight_section = -1 & c0.traversing_to = c1.traversing_to))))
@{
  (@{#0#}@.behindsec[#1#] | (#2# & (@{#0#}@.sec = @{#1#}@.from | @{#0#}@.sec = -1 & @{#0#}@.to = @{#1#}@.to)))
}@***.newMethod[behindconds, 2]

-- Actual declaration of behind mechanism. We declare behind variable, update the monitor and enforce behind relation.
-- TODO[AB]: We might consider this in a more generic setting (not only abstract model?)
--
--#define BehindDecl(c0, c1, was_behind_var, behind_var) \
--  VAR was_behind_var : boolean;   \
--      behind_var : boolean;                 \
--  INIT !was_behind_var;           \
--  TRANS next(was_behind_var) <-> behindconds(c0, c1, was_behind_var); \
--  INVAR behind_var <-> behindconds(c0, c1, was_behind_var); \
--  TRANS behind_var & c0.on_straight_section = c1.on_straight_section -> next(behind_var);
@{
    VAR #2# : boolean;
        #3# : boolean;
    INIT ! #2#;
    TRANS next(#2#) <-> @{#0#}@.behindconds[#1#,#2#];
    INVAR #3# <-> @{#0#}@.behindconds[#1#,#2#];
    TRANS #3# & @{#0#}@.sec = @{#1#}@.sec -> next(#3#);
}@***.newMethod[BehindDecl, 3]

