go_msat
@{
@(
  @{
     @(msat_check_ltlspec_bmc -k @{BMC_CNT}@.eval[0])@
     @(check_ltlspec_ic3 -i -a 1 -O 2)@
  }@*.if[@{BMC_CNT > 0}@.eval]
)@
@(
  @{
     @(msat_check_invar_bmc -i -a falsification -k @{BMC_CNT}@.eval[0])@
     @(check_invar_ic3 -i -a 1 -O 2)@
  }@*.if[@{BMC_CNT > 0}@.eval]
)@
}@.if[@{LTL_MODE}@.eval]
quit
