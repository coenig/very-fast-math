@{EnvModel_Alberto_macros.tpl}@********.include

-- TODO[AB]: Move this in parameter part
#define max_secparts_per_sec 4

MODULE main
  VAR
    -- section declarations
    sec0 : Section2Conn(max_secparts_per_sec, 0);
    sec1 : Section1Conn(max_secparts_per_sec, 1);
    --sec2 : Section1Conn(max_secparts_per_sec, 2);
    --sec3 : SectionSink(max_secparts_per_sec, 3);
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
    (@{car[i]}@.samesec[car[j]] & @{car[i]}@.secpart = @{car[0]}@.secpart)
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

  LTLSPEC ! (behind_sec(car0, car1) & F (behind_sec(car1, car0)));
  LTLSPEC ! (sec1.outgoing_connection_0 = -1 & behind_sec(car0, car1) & F (behind_sec(car1, car0)));
      
-- TODO: In general n_secparts and others parameter are actually the max index
MODULE Section2Conn(max_sec_parts, id)
  FROZENVAR
    n_secparts : 0..max_sec_parts;
    outgoing_connection_0 : -1 .. @{SECTIONS - 1}@.eval[0];
    outgoing_connection_1 : -1 .. @{SECTIONS - 1}@.eval[0];

  -- If a single connection occurs, it is on connection 0
  INIT outgoing_connection_0 = -1 -> outgoing_connection_1 = -1;

  -- there are no duplicated connections
  INIT outgoing_connection_0 != -1 -> outgoing_connection_0 != outgoing_connection_1;

  -- no selfloop
  INIT outgoing_connection_0 != id & outgoing_connection_1 != id;

MODULE Section1Conn(max_sec_parts, id)
  FROZENVAR
    n_secparts : 0..max_sec_parts;
    outgoing_connection_0 : -1 .. @{SECTIONS - 1}@.eval[0];

  -- no selfloop
  INIT outgoing_connection_0 != id;

MODULE SectionSink(max_sec_parts, id)
  FROZENVAR
    n_secparts : 0..max_sec_parts;
