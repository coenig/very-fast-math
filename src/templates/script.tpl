@{
go_msat
@{
@(
   set bmc_length @{BMC_CNT}@.eval[0]
   scengen_generate -u "@{SCENGEN_UNIVERSAL_PROPERTIES}@.printHeap" -e "@{SCENGEN_EXISTENTIAL_PROPERTIES}@.printHeap"
)@
@(
   @{
   @(
      @{
         @(msat_check_ltlspec_bmc @{-N @{BMC_NUMBER_OF_CEXS}@.eval[0]}@.if[@{BMC_NUMBER_OF_CEXS != 1}@.eval] -k @{BMC_CNT}@.eval[0])@
         @(check_ltlspec_ic3 -i -a 1 -O 2)@
      }@*.if[@{BMC_CNT > 0}@.eval]
   )@
   @(
      @{
         @(msat_check_invar_bmc -i -a falsification @{-N @{BMC_NUMBER_OF_CEXS}@.eval[0]}@.if[@{BMC_NUMBER_OF_CEXS != 1}@.eval] -k @{BMC_CNT}@.eval[0])@
         @(check_invar_ic3 -i -a 1 -O 2)@
      }@*.if[@{BMC_CNT > 0}@.eval]
   )@
   }@**.if[@{LTL_MODE}@.eval]
)@
}@***.if[@{SCENGEN_MODE}@.eval]
quit
}@.removeBlankLines