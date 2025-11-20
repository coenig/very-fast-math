@{EnvModel_Alberto_macros.tpl}@********.include

-- TODO[AB]: Move this in parameter part

MODULE main
@{
  FROZENVAR @{[sec]}@.nsecparts : 1 .. @{MAX_SEGPARTS - 1}@.eval[0];
  @{
  FROZENVAR @{[sec]}@.outconn[[con]] : -1..@{SECTIONS - 1}@.eval[0];
  INIT @{[sec]}@.outconn[[con]] != [sec]; -- Dont connect to self.
  @{
  INIT @{[sec]}@.outconn[[con]] = -1 -> @{[sec]}@.outconn[@{[con] + 1}@.eval[0]] = -1;
  }@***.if[@{[con] < MAXOUTGOINGCONNECTIONS -1}@.eval] -- Reduce topological permutations
  }@****.for[[con], 0, @{MAXOUTGOINGCONNECTIONS-1}@.eval] -- Several elements can be equal, so we have at least 1 and at most @{MAXOUTGOINGCONNECTIONS}@.eval[0] outgoing connections.
}@******.for[[sec], 0, @{SECTIONS - 1}@.eval]
  -- non-ego cars declaration

  @{
  @{car[car_num]}@.CarCoreDecl[@{SECTIONS - 1}@.eval[0], @{MAX_SEGPARTS - 1}@.eval[0]]
  }@*.for[[car_num], 0, @{NONEGOS - 1}@.eval[0]]

  -- Avoid car collisions
  INVAR 
  @{
  @{
    -- Car[i] and Car[j] are not in collision!
    !(@{car[i]}@.samesec[car[j]] & @{car[i]}@.secpart = @{car[j]}@.secpart)
  }@*.for[[j], @{[i] + 1}@.eval[0], @{NONEGOS - 1}@.eval[0],1,&]
  }@**.for[[i], 0, @{NONEGOS - 1}@.eval[0],1,&]
  TRUE

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
  
  -- reverse car:
  LTLSPEC NAME reverse_cars :=
    ! (
    -- initially the cars are ordered 0,..., n
    @{ @{ @{car[c0]}@.behindsec[car[c1]] }@*.for[ [c1], @{[c0] + 1}@.eval[0], @{NONEGOS - 1}@.eval[0],1,&]
    }@**.for[ [c0], 0, @{NONEGOS - 1}@.eval[0],1,&]
    -- then at some point order is reversed.
    F (
      @{
      @{
      @{car[c1]}@.behindsec[car[c0]] 
    }@*.for[ [c1], @{[c0] + 1}@.eval[0], @{NONEGOS - 1}@.eval[0], 1, &]
    }@**.for[ [c0], 0, @{NONEGOS - 1}@.eval[0], 1, &]
    TRUE
    )
    )
