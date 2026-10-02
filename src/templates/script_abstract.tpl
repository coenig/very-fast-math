set on_failure_script_quits 1
set pp_list cpp
go_msat
build_boolean_model
check_ltlspec_ic3 -F
quit
