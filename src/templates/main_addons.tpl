   --= CROSSING =--
   -- Remove nil from generator to activate.
@{
   -- "ANGLEGRANULARITY": "#{90}#",
   -- "MAXDISTCONNECTIONS": "#{20}#",
   -- "MAXOUTGOINGCONNECTIONS": "#{3}#",
   -- "MINDISTCONNECTIONS": "#{10}#",
   -- "EGOLESS": true,
   -- "SECTIONSMAXLENGTH": "#{100}#",
   -- "SECTIONSMINLENGTH": "#{50}#",
   -- "SEGMENTS": "#{1}#",
   -- "SEGMENTSMINLENGTH": 0,
   -- "NUMLANES": "#{1}#"
   -- "SECTIONS": "#{8}#"
   -- "SPEC": "#{ (env.veh___609___.is_on_sec_1 == 0) || (env.veh___609___.abs_pos < 40) }#"

   INIT outgoing_connection_0_of_section_0 = 1;
   INIT outgoing_connection_1_of_section_0 = 3;
   INIT outgoing_connection_2_of_section_0 = 5;
   INIT outgoing_connection_0_of_section_2 = 3;
   INIT outgoing_connection_1_of_section_2 = 5;
   INIT outgoing_connection_2_of_section_2 = 7;
   INIT outgoing_connection_0_of_section_4 = 1;
   INIT outgoing_connection_1_of_section_4 = 7;
   INIT outgoing_connection_2_of_section_4 = 5;
   INIT outgoing_connection_0_of_section_6 = 7;
   INIT outgoing_connection_1_of_section_6 = 1;
   INIT outgoing_connection_2_of_section_6 = 3;
   INIT section_0.angle = 0;
   INIT section_1.angle = 90;
   INIT section_2.angle = 270;
   INIT section_3.angle = 0;
   INIT section_4.angle = 180;
   INIT section_5.angle = 270;
   INIT section_6.angle = 90;
   INIT section_7.angle = 180;
   INIT section_4.source.y <= section_0.source.y - 6;
   INIT section_1.source.x <= section_2.source.x - 6;
   INIT veh___609___.is_on_sec_4 = 1;
   INIT veh___609___.abs_pos = 0;
   INIT veh___619___.is_on_sec_6 = 1;
   INIT veh___619___.abs_pos = 0;
   INIT veh___629___.is_on_sec_0 = 1;
   INIT veh___629___.abs_pos = 0;
   INIT veh___639___.is_on_sec_2 = 1;
   INIT veh___639___.abs_pos = 0;
   INIT section_0.drain.x = section_7.source.x;
}@***********.nil

   --= ROUNDABOUT with straight sections which can be made ROUND by the below tweak with zero-length sections =--
   -- Remove nil from generator to activate.
@{
   -- "ANGLEGRANULARITY": "#{45}#",
   -- "MAXDISTCONNECTIONS": "#{20}#",
   -- "MAXOUTGOINGCONNECTIONS": "#{2}#",
   -- "MINDISTCONNECTIONS": "#{10}#",
   -- "EGOLESS": true,
   -- "SECTIONSMAXLENGTH": "#{50}#",
   -- "SECTIONSMINLENGTH": "#{50}#",
   -- "SEGMENTS": "#{1}#",
   -- "SEGMENTSMINLENGTH": 0,
   -- "NUMLANES": "#{1}#"
   -- "SECTIONS": "#{12}#"
   -- "SPEC": "#{ env.cnt < 0 }#"

   INIT section_0.angle = 0;
   INIT section_1.angle = 90;
   INIT section_2.angle = 270;
   INIT section_3.angle = 0;
   INIT section_4.angle = 180;
   INIT section_5.angle = 270;
   INIT section_6.angle = 90;
   INIT section_7.angle = 180;
   INIT section_8.angle = 45;
   INIT section_9.angle = 315;
   INIT section_10.angle = 225;
   INIT section_11.angle = 135;
    
   INIT section_0_end != 0; -- Include this to get ROUND roundabout.
   INIT section_1_end != 0;
   INIT section_2_end != 0;
   INIT section_3_end != 0;
   INIT section_4_end != 0;
   INIT section_5_end != 0;
   INIT section_6_end != 0;
   INIT section_7_end != 0;
   INIT section_8_end = 0;
   INIT section_9_end = 0;
   INIT section_10_end = 0;
   INIT section_11_end = 0;
   
   INIT section_7.source.y = section_0.source.y - 7;
   INIT section_7.source.x = section_0.drain.x;
   INIT section_3.source.y = section_4.source.y + 7;
   INIT section_3.source.x = section_4.drain.x;
   INIT section_6.drain.y = section_5.source.y;
   INIT section_6.source.x = section_5.source.x - 7;
   INIT section_1.source.y = section_2.drain.y;
   INIT section_1.source.x = section_2.source.x - 7;
   
   INIT outgoing_connection_0_of_section_0 = 8;
   INIT outgoing_connection_0_of_section_1 = -1;
   INIT outgoing_connection_0_of_section_2 = 9;
   INIT outgoing_connection_0_of_section_3 = -1;
   INIT outgoing_connection_0_of_section_4 = 10;
   INIT outgoing_connection_0_of_section_5 = -1;
   INIT outgoing_connection_0_of_section_6 = 11;
   INIT outgoing_connection_0_of_section_7 = -1;
   INIT outgoing_connection_0_of_section_8 = 1;
   INIT outgoing_connection_0_of_section_9 = 3;
   INIT outgoing_connection_0_of_section_10 = 5;
   INIT outgoing_connection_0_of_section_11 = 7;
   INIT outgoing_connection_1_of_section_8 = 9;
   INIT outgoing_connection_1_of_section_9 = 10;
   INIT outgoing_connection_1_of_section_10 = 11;
   INIT outgoing_connection_1_of_section_11 = 8;
}@***********.nil


   --= "Round" ROUNDABOUT =--
   -- Remove nil from generator to activate.
@{
   -- "ANGLEGRANULARITY": "#{90}#",
   -- "MAXDISTCONNECTIONS": "#{20}#",
   -- "MAXOUTGOINGCONNECTIONS": "#{2}#",
   -- "MINDISTCONNECTIONS": "#{5}#",
   -- "EGOLESS": true,
   -- "SECTIONSMAXLENGTH": "#{50}#",
   -- "SECTIONSMINLENGTH": "#{50}#",
   -- "SEGMENTS": "#{1}#",
   -- "SEGMENTSMINLENGTH": 0,
   -- "NUMLANES": "#{1}#"
   -- "SECTIONS": "#{16}#"
   -- "SPEC": "#{ env.cnt < 0 }#"

   INIT section_0.angle = 0;
   INIT section_1.angle = 90;
   INIT section_2.angle = 270;
   INIT section_3.angle = 0;
   INIT section_4.angle = 180;
   INIT section_5.angle = 270;
   INIT section_6.angle = 90;
   INIT section_7.angle = 180;
   INIT section_8.angle = 90;
   INIT section_9.angle = 0;
   INIT section_10.angle = 0;
   INIT section_11.angle = 270;
   INIT section_12.angle = 270;
   INIT section_13.angle = 180;
   INIT section_14.angle = 180;
   INIT section_15.angle = 90;

   INIT section_0_end != 0;
   INIT section_1_end != 0;
   INIT section_2_end != 0;
   INIT section_3_end != 0;
   INIT section_4_end != 0;
   INIT section_5_end != 0;
   INIT section_6_end != 0;
   INIT section_7_end != 0;
   INIT section_8_end = 0;
   INIT section_9_end = 0;
   INIT section_10_end = 0;
   INIT section_11_end = 0;
   INIT section_12_end = 0;
   INIT section_13_end = 0;
   INIT section_14_end = 0;
   INIT section_15_end = 0;

   -- INIT section_7.source.x = section_0.drain.x;
   -- INIT section_3.source.x = section_4.drain.x;
   -- INIT section_6.drain.y = section_5.source.y;
   -- INIT section_2.drain.y = section_1.source.y;
   INIT section_7.source.y <= section_0.source.y - 5;
   INIT section_3.source.y >= section_4.source.y + 5;
   INIT section_6.source.x <= section_5.source.x - 5;
   INIT section_1.source.x <= section_2.source.x - 5;

   INIT outgoing_connection_0_of_section_0 = 8;
   INIT outgoing_connection_0_of_section_1 = -1;
   INIT outgoing_connection_0_of_section_2 = 10;
   INIT outgoing_connection_0_of_section_3 = -1;
   INIT outgoing_connection_0_of_section_4 = 12;
   INIT outgoing_connection_0_of_section_5 = -1;
   INIT outgoing_connection_0_of_section_6 = 14;
   INIT outgoing_connection_0_of_section_7 = -1;
   INIT outgoing_connection_0_of_section_8 = 9;
   INIT outgoing_connection_0_of_section_9 = 10;
   INIT outgoing_connection_0_of_section_10 = 11;
   INIT outgoing_connection_0_of_section_11 = 12;
   INIT outgoing_connection_0_of_section_12 = 13;
   INIT outgoing_connection_0_of_section_13 = 14;
   INIT outgoing_connection_0_of_section_14 = 15;
   INIT outgoing_connection_0_of_section_15 = 8;
--   INIT outgoing_connection_1_of_section_8 = -1;
   INIT outgoing_connection_1_of_section_9 = 1;
--   INIT outgoing_connection_1_of_section_10 = -1;
   INIT outgoing_connection_1_of_section_11 = 3;
--   INIT outgoing_connection_1_of_section_12 = -1;
   INIT outgoing_connection_1_of_section_13 = 5;
--   INIT outgoing_connection_1_of_section_14 = -1;
   INIT outgoing_connection_1_of_section_15 = 7;
}@***********.nil

   --= For Stop&Go =--
   -- Remove nil from generator to activate.
@{
@{
INIT veh___6[i]9___.lane_single;
INIT veh___6[i]9___.v <= ego.v + 3;
INVAR veh___6[i]9___.rel_pos < @{17 + 70 / 3 * NONEGOS}@.eval[0];
-- TRANS abs(next(veh___6[i]9___.a) - veh___6[i]9___.a) <= 1;
-- @{INIT ego.same_lane_as_veh_[i] & veh___6[i]9___.rel_pos > 0;}@.if[@{[i] < 5 && [i] < 2 * NONEGOS / 3}@.eval]
INIT -- ego.same_lane_as_veh_[i] & 
    veh___6[i]9___.rel_pos > 0;

@{
INVAR veh___6@{[i]-1}@.eval[0]9___.rel_pos < veh___6[i]9___.rel_pos;
ASSIGN next(veh___6@{[i]-1}@.eval[0]9___.a) := veh___6[i]9___.a;
}@.if[@{[i]>0}@.eval]

}@***.for[[i], 0, @{NONEGOS - 1}@.eval]
INIT ego_lane_0;

INVAR ego.gaps___619___.i_agent_front = 0;

INVAR section_0_segment_0_min_lane = section_0_segment_1_min_lane;
INVAR section_0_segment_1_min_lane = section_0_segment_2_min_lane;
INVAR section_0_segment_2_max_lane = 0;
}@***********.nil


   --= General "Section" Stuff =--
INIT env.section_1.angle != 0;
INIT env.section_2.angle != 0;
INIT env.ego.on_straight_section = 0;
INIT env.veh___619___.abs_pos < env.section_0_end;
INIT env.veh___629___.abs_pos < env.section_0_end;
INIT env.veh___619___.on_straight_section = 0;
INIT env.veh___629___.on_straight_section = 0;
INIT env.ego.on_straight_section = 0;
INVAR env.ego.v <= 18;
