@{
This is an example script on how to create Road Graph images with vfm.

Use this command from the bin folder to expand the script:
./vfm.exe --execute-script --rootdir .

On the NT, it can be expanded via CLI:
pace mc --execute_script

If this script template file is present in the "templates" folder, it will be used in case there is no explicit file provided. 
An explicit script can be provided as *root_dir*/script.txt. *root_dir* defaults to /workspace on the NT.

The syntax of a single road graph instance is:
"((x_src, y_src), angle, (max_lanes, section_length, ((seg_1_min_lane, seg_1_max_lane, seg_1_begin), ..., ((seg_i_min_lane, seg_i_max_lane, seg_i_begin)))))"

Below follows an example of how a road graph image can be created and stored as "mytest.png". 
}@.nil



@{ ((0, 0), 0, (4, 50, ((0, 5, 0)))) }@.createRoadGraph[0]
@{ ((90, 20), 45, (4, 50, ((0, 5, 0), (0, 3, 30)))) }@.createRoadGraph[1]
@{ ((90, -20), 0, (4, 50, ((0, 5, 0)))) }@.createRoadGraph[2]
@{0}@.connectRoadGraphTo[1]
@{0}@.connectRoadGraphTo[2]

@{
@{@WIDTH_FACTOR_NON_INFINITE = 1}@.eval
@{@OFFSET_Y_NON_INFINITE = 10}@.eval
}@.nil

@{0}@.storeRoadGraph[mytest.png]

Variable values for image creation I used:
WIDTH_FACTOR_NON_INFINITE = @{WIDTH_FACTOR_NON_INFINITE}@.eval[0]
HEIGHT_FACTOR_NON_INFINITE = @{HEIGHT_FACTOR_NON_INFINITE}@.eval[0]
OFFSET_X_NON_INFINITE = @{OFFSET_X_NON_INFINITE}@.eval[0]
OFFSET_Y_NON_INFINITE = @{OFFSET_Y_NON_INFINITE}@.eval[0]
DIMENSION_X = @{DIMENSION_X}@.eval[0]
DIMENSION_Y = @{DIMENSION_Y}@.eval[0]
