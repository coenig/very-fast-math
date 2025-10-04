@{
Use this command from the bin folder to expand the script:
./vfm.exe --execute-script --rootdir .

The syntax of a road graph is: 
"((x_src, y_src), angle, (max_lanes, section_length, ((seg_1_min_lane, seg_1_max_lane, seg_1_begin), ..., ((seg_i_min_lane, seg_i_max_lane, seg_i_begin)))))"
}@.nil

@{ ((0, 0), 0, (4, 50, ((0, 5, 0)))) }@.createRoadGraph[0]
@{ ((90, 20), 0, (4, 50, ((0, 5, 0), (0, 3, 30)))) }@.createRoadGraph[1]
@{ ((90, -20), 0, (4, 50, ((0, 5, 0)))) }@.createRoadGraph[2]
@{0}@.connectRoadGraphTo[1]
@{0}@.connectRoadGraphTo[2]
@{0}@.storeRoadGraph[mytest]
