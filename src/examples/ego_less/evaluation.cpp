// #vfm-insert-includes

// #vfm-begin
// #vfm-ltl-spec[[ true ]]
// #vfm-insert-associativity-pretext
struct LcFastlaneConditions
{
   bool cond26_all_conditions_fulfilled_raw{ false };
   bool cond30_all_conditions_fulfilled{ false };
};

// lanechange abort
struct LcAbortConditions
{
   bool cond26_all_conditions_fulfilled_raw{ false };
   bool cond30_all_conditions_fulfilled{ false };
};

void checkLCConditionsFastLane(
    LcFastlaneConditions& flCond,
    LcAbortConditions& abCond
)
{
    // #vfm-gencode-begin[[ condition=false ]]
   flCond.cond26_all_conditions_fulfilled_raw = false;

    abCond.cond26_all_conditions_fulfilled_raw = false;
    // #vfm-gencode-end

}
// #vfm-end
