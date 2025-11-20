@{EnvModel_Alberto_macros.tpl}@********.include

-- TODO[AB]: Move this in parameter part
#define max_secparts_per_sec 4

MODULE main
@{
  FROZENVAR @{[sec]}@.nsecparts : 0 .. max_secparts_per_sec;
  @{
  FROZENVAR @{[sec]}@.outconn[[con]] : -1..@{SECTIONS - 1}@.eval[0];
  INIT @{[sec]}@.outconn[[con]] != [sec]; -- Don't connect to self.
  @{
  INIT @{[sec]}@.outconn[[con]] = -1 -> @{[sec]}@.outconn[@{[con] + 1}@.eval[0]] = -1;
  }@***.if[@{[con] < MAXOUTGOINGCONNECTIONS -1}@.eval] -- Reduce topological permutations
  }@****.for[[con], 0, @{MAXOUTGOINGCONNECTIONS-1}@.eval] -- Several elements can be equal, so we have at least 1 and at most @{MAXOUTGOINGCONNECTIONS}@.eval[0] outgoing connections.
}@******.for[[sec], 0, @{SECTIONS - 1}@.eval]
  -- non-ego cars declaration

  -- part of declaration indipendent of interaction declared with macros.
  -- also control movement from junctions to sections

  @{
  @{car[car_num]}@.CarCoreDecl[@{SECTIONS - 1}@.eval[0], max_secparts_per_sec]
  }@*.for[[car_num], 0, @{NONEGOS - 1}@.eval[0]]

  -- Avoid car collisions
  INVAR !
  @{
  @{
    (@{car[i]}@.samesec[car[j]] & @{car[i]}@.secpart = @{car[j]}@.secpart)
  }@*.for[[j], @{i + 1}@.eval[0], @{NONEGOS - 1}@.eval[0],1,&]
  }@**.for[[i], 0, @{NONEGOS - 1}@.eval[0],1,&]

  DEFINE
    @{
    @{
    car[i]_will_travese_from_sec[j] := @{car[i]}@.willtraverse[[j]];
    }@*.for[[i], 0, @{NONEGOS - 1}@.eval[0]]
    }@*.for[[j], 0, @{SECTIONS - 1}@.eval[0]]

  @{
  @{
  -- Transitions for cars inside the sections (assuming they do not pass to next section)
  @{car[i]}@.CarTransSectInner[[j]];
  -- Transition representing junction entering
  @{car[i]}@.CarTransSectJunc[[j]];
  }@*.for[[i], 0, @{NONEGOS - 1}@.eval[0]]
  }@*.for[[j], 0, @{SECTIONS - 1}@.eval[0]]

  @{
  @{
  @{
    @{car[i]}@.BehindDecl[car[j],was_behind_car[i]_car[j],behind_car[i]_car[j]]
  }@*.if[@{[i] != [j]}@.eval]
  }@**.for[[j], 0, @{NONEGOS - 1}@.eval[0]]
  }@**.for[[i], 0, @{NONEGOS - 1}@.eval[0]]
