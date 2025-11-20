@{EnvModel_Alberto_macros.tpl}@********.include

#define max_sec_id 1
#define max_secparts_per_sec 4

--ヤッター！
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
  @{car[car_num]}@.CarCoreDecl[max_sec_id, max_secparts_per_sec]
  }@*.for[[car_num], 0, @{NONEGOS - 1}@.eval[0]]
  @{
  @{
  INVAR ! (@{car[i]}@.samesec[car[j]] & @{car[i]}@.secpart = @{car[0]}@.secpart);
  }@*.for[[j], @{i + 1}@.eval[0], @{NONEGOS - 1}@.eval[0]]
  }@*.for[[i], 0, @{NONEGOS - 1}@.eval[0]]

  DEFINE
    @{
    @{
    car[i]_will_travese_from_sec[j] := @{car[i]}@.willtraverse[[j]];
    }@*.for[[i], 0, @{NONEGOS - 1}@.eval[0]]
    }@*.for[[j], 0, @{SECTIONS - 1}@.eval[0]]

  @{
  @{
  -- Transitions for cars inside the sections (assuming they do not pass to next section)
  @{car[i]}@.CarTransSectInner[sec[j], [j]];
  -- Transition representing junction entering
  @{car[i]}@.CarTransSectJunc[sec[j], [j]];
  }@*.for[[i], 0, @{NONEGOS - 1}@.eval[0]]
  }@*.for[[j], 0, @{SECTIONS - 1}@.eval[0]]

  BehindDecl(car0, car1, was_behind_car0_car1, behind_car0_car1)
  BehindDecl(car1, car0, was_behind_car1_car0, behind_car1_car0)
  --TRANS behind_car0_car1 & car0.on_straight_section = car1.on_straight_section -> next(behind_car0_car1);
  --TRANS behind_car1_car0 & car1.on_straight_section = car0.on_straight_section -> next(behind_car1_car0);

#define INCLUDE_BOUND_COUNTER 0 
#if INCLUDE_BOUND_COUNTER
#define bound_counter_upper_bound
  VAR bound_counter : 0 .. bound_counter_upper_bound;

  INIT bound_counter = 0;
  TRANS next(bound_counter) = ((bound_counter = bound_counter_upper_bound) ? (bound_counter_upper_bound) : (bound_counter + 1));
#endif
  --INIT behind_sec(car0, car1) & sec0.n_secparts = 2 & sec1.n_secparts = 2;
  --INVARSPEC  !behind_sec(car1, car0);
  LTLSPEC ! (behind_sec(car0, car1) & F (behind_sec(car1, car0)));
  LTLSPEC ! (sec1.outgoing_connection_0 = -1 & behind_sec(car0, car1) & F (behind_sec(car1, car0)));
      
-- TODO: In general n_secparts and others parameter are actually the max index
MODULE Section2Conn(max_sec_parts, id)
  FROZENVAR
    n_secparts : 0..max_sec_parts;
    outgoing_connection_0 : -1 .. max_sec_id;
    outgoing_connection_1 : -1 .. max_sec_id;

  -- If a single connection occurs, it is on connection 0
  INIT outgoing_connection_0 = -1 -> outgoing_connection_1 = -1;

  -- there are no duplicated connections
  INIT outgoing_connection_0 != -1 -> outgoing_connection_0 != outgoing_connection_1;

  -- no selfloop
  INIT outgoing_connection_0 != id & outgoing_connection_1 != id;

MODULE Section1Conn(max_sec_parts, id)
  FROZENVAR
    n_secparts : 0..max_sec_parts;
    outgoing_connection_0 : -1 .. max_sec_id;

  -- no selfloop
  INIT outgoing_connection_0 != id;

MODULE SectionSink(max_sec_parts, id)
  FROZENVAR
    n_secparts : 0..max_sec_parts;
