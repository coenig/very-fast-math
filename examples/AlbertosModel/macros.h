#define behind_sec(c0, c1) \
  (same_sec(c0, c1) & (c0.on_secpart < c1.on_secpart))

#define same_sec(c0, c1) \
  (c0.on_straight_section != -1 & c0.on_straight_section = c1.on_straight_section)

#define car_at_end_of_section(c0, sec_id, sec_n_seg) \
  (c0.on_straight_section = sec_id & c0.on_secpart = sec_n_seg)


#define junction_big_or2(c0, sec0) \
  (                                 \
    (junction_big_or1(c0, sec0)) |  \
    (next(c0.traversing_to) = sec0.outgoing_connection_1)   \
  )
  
#define junction_big_or1(c0, sec0) \
  (next(c0.traversing_to) = sec0.outgoing_connection_0)

#define junction_big_or0(c0, sec0) \
  (FALSE)

#define CarCoreDecl(car_name, n_sec, secparts_per_sec) \
  VAR \
    car_name.on_secpart : -1 .. secparts_per_sec;               \
    car_name.on_straight_section : -1 .. n_sec; \
    car_name.traversing_from : -1 .. n_sec; \
    car_name.traversing_to : -1 .. n_sec; \
  INIT car_name.traversing_to = -1 & car_name.traversing_from = -1; \
  INIT car_name.on_secpart != -1;                                   \
  TRANS car_name.on_straight_section = -1 ->                                        \
    (next(car_name.on_straight_section) = -1 & next(car_name.traversing_to) = car_name.traversing_to & next(car_name.traversing_from) = car_name.traversing_from & next(car_name.on_secpart = -1)) | \
    (next(car_name.on_straight_section) = car_name.traversing_to & next(car_name.traversing_to) = -1 & next(car_name.traversing_from) = -1 & next(car_name.on_secpart) = 0); \

#define CAR_CAN_PROGRESS_WITH_MULTIPLE_STEPS 0
#ifndef CAR_CAN_PROGRESS_WITH_MULTIPLE_STEPS
  #error "Please specify whether or not the car can make more than one step in a single transition"
#endif

#if CAR_CAN_PROGRESS_WITH_MULTIPLE_STEPS
#define _do_progress_secpart(c0, sec0) \
  (next(c0.on_secpart) >= c0.on_secpart)
#define _can_traverse(c0, sec0)  \
  (TRUE)
#else
#define _do_progress_secpart(c0, sec0) \
  (c0.on_secpart < sec0.n_secparts & next(c0.on_secpart) = c0.on_secpart + 1)
#define _can_traverse(c0, sec0) \
  (c0.on_secpart = sec0.n_secparts)
#endif

#define CarTransSectInner(c0, sec0, sec0_id) \
  INVAR (c0.on_straight_section = sec0_id) -> c0.on_secpart >= 0 & c0.on_secpart <= sec0.n_secparts; \
  TRANS (c0.on_straight_section = sec0_id) & next(c0.on_straight_section) != -1 ->  \
    (next(c0.on_straight_section = sec0_id) & (next(c0.on_secpart) = c0.on_secpart | _do_progress_secpart(c0, sec0)) & \
     next(c0.on_secpart) <= sec0.n_secparts & \
     next(c0.traversing_to) = -1 & next(c0.traversing_from) = -1)

#define will_traverse(c0, sec0_id) \
  (c0.on_straight_section = sec0_id & next(c0.on_straight_section) = -1)

#define CarTransSectJunc(c0, sec0, sec0_id, junc_big_or) \
  TRANS will_traverse(c0, sec0_id) -> (_can_traverse(c0, sec0) & next(c0.on_secpart) = -1 & next(c0.traversing_from) = sec0_id & next(c0.traversing_to) != -1 & (junc_big_or))

#define CarTransSectJunc2(c0, sec0, sec0_id) \
  CarTransSectJunc(c0, sec0, sec0_id, junction_big_or2(c0, sec0))

#define CarTransSectJunc1(c0, sec0, sec0_id) \
  CarTransSectJunc(c0, sec0, sec0_id, junction_big_or1(c0, sec0))

#define CarTransSectJunc0(c0, sec0, sec0_id) \
  CarTransSectJunc(c0, sec0, sec0_id, junction_big_or0(c0, sec0))

#define behind_conds(c0, c1, was_behind_var) \
  (((behind_sec(c0, c1)) | \
        (was_behind_var & (c0.on_straight_section = c1.traversing_from | \
                           c0.on_straight_section = -1 & c0.traversing_to = c1.traversing_to))))

#define BehindDecl(c0, c1, was_behind_var, behind_var) \
  VAR was_behind_var : boolean;   \
      behind_var : boolean;                 \
  INIT !was_behind_var;           \
  TRANS next(was_behind_var) <-> behind_conds(c0, c1, was_behind_var); \
  INVAR behind_var <-> behind_conds(c0, c1, was_behind_var); \
  TRANS behind_var & c0.on_straight_section = c1.on_straight_section -> next(behind_var);
