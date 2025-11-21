@{EnvModel_Abstract_Macros.tpl}@********.include

MODULE EnvModel
  DEFINE ego_lane_crossing := FALSE;
@{EnvModel_Common_Sections.tpl}@********.include
@{
  FROZENVAR @{[sec]}@.nsecparts : 1 .. @{MAXSECPARTS - 1}@.eval[0];
}@******.for[[sec], 0, @{SECTIONS - 1}@.eval]
  -- non-ego cars declaration

  @{
  @{car[car_num]}@.CarCoreDecl[@{SECTIONS - 1}@.eval[0], @{MAXSECPARTS - 1}@.eval[0]]
  }@*.for[[car_num], 0, @{NONEGOS - 1}@.eval[0]]

  -- Avoid car collisions
  INVAR 
  @{
  @{
    -- Car[i] and Car[j] are not in collision!
    !(@{car[i]}@.samesec[car[j]] & @{car[i]}@.secpart = @{car[j]}@.secpart)
  }@*.for[[j], @{[i] + 1}@.eval[0], @{NONEGOS - 1}@.eval[0],1,&]
  }@**.for[[i], 0, @{NONEGOS - 1}@.eval[0],1,&]
  TRUE;

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
     --initially the cars are ordered 0,..., n
    @{@{car[i]}@.behindsec[car@{[i] + 1}@.eval[0]]}@*.for[[i], 0, @{NONEGOS - 2}@.eval[0],1,&] 
    & 
    -- then at some point order is reversed.
    F (@{@{car@{[i]+1}@.eval[0]}@.behindsec[car[i]]}@*.for[[i], 0, @{NONEGOS - 2}@.eval[0],1,&])

  )

  VAR
   -- TODO: counter as integer slow down verification!!!
   cnt : -1..1;

   -- "cond" variables are only there to mock the interface towards the BP
   ego.flCond_full : boolean; -- conditions for lane change to fast lane (lc allowed and desired)
   ego.slCond_full : boolean; -- conditions for lane change to slow lane (lc allowed and desired)
   ego.abCond_full : boolean; -- conditions for abort of lane change 
   -- EO "cond"
