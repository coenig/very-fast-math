@{EnvModel_Abstract_Macros.tpl}@********.include

MODULE EnvModel
  DEFINE ego_lane_crossing := FALSE;
@{EnvModel_Common_Sections.tpl}@********.include
@{
  -- Abstract environment specific variable declaration. Split the sections in section parts.
  FROZENVAR @{[sec]}@.nsecparts : 1 .. @{MAXSECPARTS - 1}@.eval[0];
}@******.for[[sec], 0, @{SECTIONS - 1}@.eval]
  -- non-ego cars declaration

  @{
  @{[car_num]}@.CarCoreDecl[@{SECTIONS - 1}@.eval[0], @{MAXSECPARTS - 1}@.eval[0]]
  }@*.for[[car_num], 0, @{NONEGOS - 1}@.eval[0]]

  -- Avoid car collisions
  INVAR 
  @{
  @{
    -- Car[i] and Car[j] are not in collision!
    !(@{[i]}@.samesec[[j]] & @{[i]}@.secpart = @{[j]}@.secpart)
  }@*.for[[j], @{[i] + 1}@.eval[0], @{NONEGOS - 1}@.eval[0],1,&]
  }@**.for[[i], 0, @{NONEGOS - 1}@.eval[0],1,&]
  TRUE;

  DEFINE
    @{
    @{
    @{[i]}@.car_will_travese_from_sec[j] := @{[i]}@.willtraverse[[j]];
    }@*.for[[i], 0, @{NONEGOS - 1}@.eval[0]]
    }@*.for[[j], 0, @{SECTIONS - 1}@.eval[0]]

  @{
  @{
  -- Transitions for cars inside the sections (assuming they do not pass to next section)
  @{[i]}@.CarTransSectInner[[j]];
  -- Transition representing junction entering
  @{[i]}@.CarTransSectJunc[[j]];
  }@*.for[[i], 0, @{NONEGOS - 1}@.eval[0]]
  }@*.for[[j], 0, @{SECTIONS - 1}@.eval[0]]

  @{
  @{
  @{
    @{[i]}@.BehindDecl[[j],was_behind_[i]_[j],behind_[i]_[j]]
  }@*.if[@{[i] != [j]}@.eval]
  }@**.for[[j], 0, @{NONEGOS - 1}@.eval[0]]
  }@**.for[[i], 0, @{NONEGOS - 1}@.eval[0]]
  
  VAR
   -- TODO: counter as integer slow down verification!!!
   cnt : -1..1;

   -- "cond" variables are only there to mock the interface towards the BP
   ego.flCond_full : boolean; -- conditions for lane change to fast lane (lc allowed and desired)
   ego.slCond_full : boolean; -- conditions for lane change to slow lane (lc allowed and desired)
   ego.abCond_full : boolean; -- conditions for abort of lane change 
   -- EO "cond"
