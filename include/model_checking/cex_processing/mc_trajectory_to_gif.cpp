//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/cex_processing/mc_trajectory_visualizers.h"
#include "model_checking/counterexample_replay.h"
#include <thread>

using namespace vfm;
using namespace mc;
using namespace trajectory_generator;

const auto VARIABLES_TO_BE_PAINTED = std::make_shared<std::vector<PainterVariableDescription>>(std::vector<PainterVariableDescription>
   { // 3: int, 2: enum, 0: bool, 1: float
      //{ "debug.ego.a",                                  { 3, -11 } },
      //{ "ego.v",                                        { 3, -11 } },
      //{ "ego.abs_pos",                                        { 3, -11 } },

      //{ "REGEX:ego.gaps___6.*9___.s_dist_front",              { 3, -11 } },
      //{ "REGEX:ego.gaps___6.*9___.s_dist_rear",               { 3, -11 } },

      //{ "ego_lane_b0",                                  { 0, -11 } },
      //{ "ego_lane_b1",                                  { 0, -11 } },
      //{ "ego_lane_b2",                                  { 0, -11 } },
      //{ "ego_lane_b3",                                  { 0, -11 } },
      //{ "ego_lane_b4",                                  { 0, -11 } },
      //{ "ego.on_lane",                                  { 3, -11 } },
      //{ "env.ego.on_lane",                                  { 3, -11 } },
      //{ "env.env_state.max_lane_id",                                  { 3, -11 } },

      //{ R"(planner."flCond.cond0_lc_conditions_evaluated")",   { 0, -11 } },
      //{ R"(planner."flCond.cond10_t_action_ok")",   { 0, -11 } },
      //{ R"(planner."flCond.cond11_enable_lanechanges")",   { 0, -11 } },
      //{ R"(planner."flCond.cond12_current_lane_is_merge_lane")",   { 0, -11 } },
      //{ R"(planner."flCond.cond14_shadow_agent_ahead")",   { 0, -11 } },
      //{ R"(planner."flCond.cond15_escape_slowlane_deadlock")",   { 0, -11 } },

      //{ R"(env.ego.slow_lane_deadlock_active)",   { 0, -11 } },

      //{ R"(planner."flCond.cond16_forced_merge")",   { 0, -11 } },
      //{ R"(planner."flCond.cond18_driver_lanechange_request")",   { 0, -11 } },
      //{ R"(planner."flCond.cond19_navigation_system_request")",   { 0, -11 } },
      //{ R"(planner."flCond.cond20_reactive_conditions_fulfilled")",   { 0, -11 } },
      //{ R"(planner."flCond.cond22_safety_conditions_fulfilled")",   { 0, -11 } },
      //{ R"(planner."flCond.cond24_external_conditions_fulfilled")",   { 0, -11 } },
      { R"(planner."abCond.cond26_all_conditions_fulfilled_raw")",   { 0, -11 } },
      //{ R"(env.ego.flCond_full)",   { 0, -11 } },
      //{ R"(env.ego.mode)",   { 2, -11 } },
      //{ R"(planner."flCond.cond1_lc_trigger_dist_reached")",   { 0, -11 } },
      //{ R"(planner."flCond.cond2_ego_faster_than_lead")",   { 0, -11 } },
      //{ R"(planner."flCond.cond3_lane_speed_allows_overtake")",   { 0, -11 } },
      //{ R"(planner."flCond.cond4_lead_not_much_faster_than_ego")",   { 0, -11 } },
      //{ R"(planner."flCond.cond5_ax_acf_target_lane_same_or_higher")",   { 0, -11 } },
      //{ R"(planner."flCond.cond6_target_rear_vehicle_mild_braking")",   { 0, -11 } },
      //{ R"(planner."flCond.cond7_dist_to_target_front_vehicle_large_enough")",   { 0, -11 } },
      //{ R"(planner."flCond.cond8_dist_to_target_rear_vehicle_large_enough")",   { 0, -11 } },

      //{ R"(planner."abCond.cond0_lc_abort_evaluated")",   { 0, -11 } },
      //{ R"(planner."abCond.cond10_before_point_of_no_return")",   { 0, -11 } },
      //{ R"(planner."abCond.cond20_reactive_conditions_fulfilled")",   { 0, -11 } },
      //{ R"(planner."abCond.cond26_all_conditions_fulfilled_raw")",   { 0, -11 } },
      //{ R"(env.ego.abCond_full)",   { 0, -11 } },
      //{ R"(env.ego.flCond_full)",   { 0, -11 } },
      //{ R"(env.ego_lane_move_up)",   { 3, -11 } },
      //{ R"(env.ego_lane_move_down)",   { 3, -11 } },

      //{ R"(planner."abCond.cond1_target_rear_vehicle_hard_braking")",   { 0, -11 } },
      //{ R"(planner."abCond.cond2_dist_to_target_front_vehicle_small")",   { 0, -11 } },
      //{ R"(planner."abCond.cond3_dist_to_target_rear_vehicle_small")",   { 0, -11 } },
      //{ R"(REGEX:.*min_dist_target_front")",   { 3, -11 } },
      //{ R"(REGEX:.*min_dist_target_front_dyn")",   { 3, -11 } },
      //{ R"(REGEX:.*abCond2DistToTargetFrontVehicleSmall.v_rel_tmp")",   { 3, -11 } },
      //{ R"(REGEX:.*getDynamicMinDistanceFront.part1.1")",   { 3, -11 } },
      //{ R"(REGEX:.*getDynamicMinDistanceFront.part2.1")",   { 3, -11 } },
      //{ R"(REGEX:veh___6.*9___.rel_pos)",   { 3, -11 } },

      { R"(REGEX:.*env.ego..*)",   { 3, -11 } },
      { R"(REGEX:.*rlc.*)",   { 1, -11 } },

      //{ R"(ego.right_of_veh_1_lane)",   { 0, -11 } },
      //{ R"(veh_length)",   { 3, -11 } },
      //{ R"(veh___619___.lane_move_up)",   { 0, -11 } },
      //{ R"(veh___619___.lane_move_down)",   { 0, -11 } },

      //{ R"(planner."!{params.enable_lane_change_driver_request}")",   { 0, -11 } },
      //{ R"(planner."!{abCond2DistToTargetFrontVehicleSmall.delta_v}")",   { 3, -11 } },
      //{ R"(planner."!{abCond2DistToTargetFrontVehicleSmall.delta_dist}")",   { 3, -11 } },
      //{ R"(planner."!{TEMPORARRAY5}")",   { 3, -11 } },
      //{ R"(planner."!{tar_dir}")",   { 2, -11 } },
      //{ R"(env.tar_dir)",   { 2, -11 } },

      //{ "REGEX:veh___6.*9___.rel_pos",                         { 3, -11 } },
      //{ "veh___609___.lane_b0",                         { 0, -11 } },
      //{ "veh___609___.lane_b1",                         { 0, -11 } },
      //{ "veh___609___.lane_b2",                         { 0, -11 } },
      //{ "veh___609___.lane_b3",                         { 0, -11 } },
      //{ "veh___609___.lane_b4",                         { 0, -11 } },
      //{ "veh___609___.on_lane",                         { 3, -11 } },
      //{ "veh___619___.lane_b0",                         { 0, -11 } },
      //{ "veh___619___.lane_b1",                         { 0, -11 } },
      //{ "veh___619___.lane_b2",                         { 0, -11 } },
      //{ "veh___619___.lane_b3",                         { 0, -11 } },
      //{ "veh___619___.lane_b4",                         { 0, -11 } },
      //{ "veh___619___.on_lane",                         { 3, -11 } },
      //{ "veh___649___.on_lane",                         { 3, -11 } },
      //{ "veh___649___.on_lane_min",                     { 3, -11 } },
      //{ "veh___649___.on_lane_max",                     { 3, -11 } },
      //{ "debug.ego.gaps___609___.lane_availability",    { 3, -11 } },
      //{ "debug.ego.gaps___619___.lane_availability",    { 3, -11 } },
      //{ "debug.ego.gaps___629___.lane_availability",    { 3, -11 } },
      //{ "segment_0_min_lane",                     { 3, -11 } },
      //{ "segment_0_max_lane",                     { 3, -11 } },
      //{ "segment_1_min_lane",                     { 3, -11 } },
      //{ "segment_1_max_lane",                     { 3, -11 } },
      //{ "segment_2_min_lane",                     { 3, -11 } },
      //{ "segment_2_max_lane",                     { 3, -11 } },
      //{ "segment_3_min_lane",                     { 3, -11 } },
      //{ "segment_3_max_lane",                     { 3, -11 } },
      //{ "segment_0_pos_begin",                     { 3, -11 } },
      //{ "segment_1_pos_begin",                     { 3, -11 } },
      //{ "segment_2_pos_begin",                     { 3, -11 } },
      //{ "segment_3_pos_begin",                     { 3, -11 } },
      //{ "ego.gaps___629___.s_dist_rear",                { 3, -11 } },
      //{ "debug.ego_pressured_from_ahead_on_left_lane",  { 0, -11 } },
      //{ "debug.ego_pressured_from_ahead_on_own_lane",   { 0, -11 } },
      //{ "debug.ego_pressured_from_ahead_on_right_lane", { 0, -11 } },
      //{ "ego.has_close_vehicle_on_left_left_lane",      { 0, -11 } },
      //{ "ego.has_close_vehicle_on_right_right_lane",    { 0, -11 } },
      //{ "ego.flCond_full",                              { 0, -11 } },
      //{ "ego_timer",                                    { 3, -11 } },
      //{ "ego.lc_direction",                             { 2, -11 } },
      //{ "ego.mode",                                     { 2, -11 } },
      //{ "debug.gaps___609___.a_front",                                     { 3, -11 } },
      //{ "debug.gaps___619___.a_front",                                     { 3, -11 } },
      //{ "debug.gaps___629___.a_front",                                     { 3, -11 } },
      //
      //{ R"(ego.change_lane_now)",                          { 3, -11 } },
      //{ R"(REGEX:ego_lane_move_.*)",                          { 3, -11 } },
      //{ R"(REGEX:ego.mode)",                          { 2, -11 } },
      //{ R"(REGEX:veh___6.*9___.change_lane_now)",                          { 0, -11 } },
      //{ R"(REGEX:debug.ego..*_of_veh_.*_lane)",                          { 0, -11 } },
      //{ R"(REGEX:debug.*pressured.*)",                          { 0, -11 } },

      //{ R"(env.tar_dir)",                                                                     { 2, -11 } },
      //{ R"(planner."!{tar_dir}")",                                                                     { 2, -11 } },
      ////{ R"(planner."abCond2DistToTargetFrontVehicleSmall.min_dist_target_front")",         { 3, -11 } },
      //{ R"(REGEX:env.ego.gaps___6.*9___.i_agent.*)",                           { 3, -11 } },
      //{ R"(REGEX:env.ego.gaps___6.*9___.s_dist.*)",                           { 3, -11 } },
      //{ R"(planner."!{ego.gaps___609___.i_agent_front}")",                           { 3, -11 } },
      //{ R"(planner."ego.gaps___6tar_dir9___.i_agent_front")",                           { 3, -11 } },

      //{ "debug.crash",                                  { 0, -11 } },
   }
);

std::shared_ptr<RoadGraph> LiveSimGenerator::getRoadGraphTopologyFrom(const MCTrace& trace)
{
   if (trace.empty()) Failable::getSingleton()->addError("Received empty trace for 'getRoadGraphFrom'.");

   const auto first_state{ trace.at(0).second };

   const auto segment_begin_name = [](const int sec, const int seg) -> std::string { return "section_" + std::to_string(sec) + "_segment_" + std::to_string(seg) + "_pos_begin"; };
   const auto segment_min_lane_name = [](const int sec, const int seg) -> std::string { return "section_" + std::to_string(sec) + "_segment_" + std::to_string(seg) + "_min_lane"; };
   const auto segment_max_lane_name = [](const int sec, const int seg) -> std::string { return "section_" + std::to_string(sec) + "_segment_" + std::to_string(seg) + "_max_lane"; };
   const auto connection = [](const int sec, const int con) -> std::string { return "env.outgoing_connection_" + std::to_string(con) + "_of_section_" + std::to_string(sec); };

   std::vector<std::shared_ptr<RoadGraph>> road_graphs{};

   for (int sec = 0; first_state.count(segment_begin_name(sec, 0)); sec++) {
      road_graphs.push_back(std::make_shared<RoadGraph>(sec));
      StraightRoadSection lane_structure{ std::stoi(first_state.at("num_lanes")), std::stof(first_state.at("section_" + std::to_string(sec) + "_end")) };
      lane_structure.setNumLanes(std::stoi(trace.at(0).second.at("num_lanes")));

      for (int seg = 0; first_state.count(segment_min_lane_name(sec, seg)); seg++) {
         lane_structure.addLaneSegment({
            std::stof(first_state.at(segment_begin_name(sec, seg))),
            (lane_structure.getNumLanes() - 1 - std::stoi(first_state.at(segment_max_lane_name(sec, seg)))) * 2, // Remove "* 2"...
            (lane_structure.getNumLanes() - 1 - std::stoi(first_state.at(segment_min_lane_name(sec, seg)))) * 2, // ...to activate hard shoulders.
            }
            );
      }

      road_graphs[sec]->setOriginPoint({
         std::stof(first_state.at("section_" + std::to_string(sec) + ".source.x")),
         std::stof(first_state.at("section_" + std::to_string(sec) + ".source.y")) });

      auto angle_raw = std::stof(first_state.at("section_" + std::to_string(sec) + ".angle"));
      auto angle = 2.0 * 3.1415 * angle_raw / 360.0;
      road_graphs[sec]->setAngle(angle);
      road_graphs[sec]->setMyRoad(lane_structure);
   }

   CarPars c{ 0, 0, 0, vfm::HighwayImage::EGO_MOCK_ID };
   road_graphs[0]->getMyRoad().setEgo(std::make_shared<CarPars>());

   for (int sec = 0; first_state.count(segment_begin_name(sec, 0)); sec++) {
      for (int connect = 0; first_state.count(connection(sec, connect)); connect++) {
         int successor = std::stof(first_state.at(connection(sec, connect)));

         if (successor >= 0) { // -1 is code for "not present".
            road_graphs[sec]->addSuccessor(road_graphs.at(successor));
         }
      }
   }

   return road_graphs[0];
}

void vfm::mc::trajectory_generator::LiveSimGenerator::equipRoadGraphWithCars(
   const std::shared_ptr<RoadGraph> r, const size_t trajectory_index, const double x_scaling)
{
   const auto& ego_trajectory = m_trajectory_provider.getEgoTrajectory();
   const auto& current_ego = ego_trajectory[trajectory_index];

   r->cleanFromAllCars();

   //const float car_lane, const float car_rel_pos, const int car_velocity, const int car_id
   const auto ego = std::make_shared<CarPars>(
      r->getMyRoad().getNumLanes() - 1 + current_ego.second.at(PossibleParameter::pos_y) / mc::trajectory_generator::LANE_WIDTH, // Lane
      current_ego.second.at(PossibleParameter::pos_x),                                                                           // Position
      current_ego.second.at(PossibleParameter::vel_x) / x_scaling,                                                               // Velocity
      vfm::HighwayImage::EGO_MOCK_ID                                                                                             // ID
   );

   r->findSectionWithID(0)->getMyRoad().setEgo(ego); // TODO: Use correct section, once available.

   auto& vehicle_names_without_ego = m_trajectory_provider.getVehicleNames(true);

   for (const auto& vehicle_name : vehicle_names_without_ego) {
      const auto& traj_pos = m_trajectory_provider.getVehicleTrajectory(vehicle_name)[trajectory_index];
      const int on_straight_section{ (int) traj_pos.second.at(PossibleParameter::on_straight_section) };
      const int traversion_from{ (int) traj_pos.second.at(PossibleParameter::traversion_from) };
      const int traversion_to{ (int) traj_pos.second.at(PossibleParameter::traversion_to) };
      const int vehicle_index{ StaticHelper::extractIntegerAfterSubstring(
         StaticHelper::replaceAll(vehicle_name, "9___", "____"), // Avoid getting 49 instead of 4 as index.
         "veh___6") };

      const CarPars veh{
         (float) (r->getMyRoad().getNumLanes() - 1 + traj_pos.second.at(PossibleParameter::pos_y) / mc::trajectory_generator::LANE_WIDTH), // Lane
         (float) (traj_pos.second.at(PossibleParameter::pos_x)),                                                                           // Position
         (int) (traj_pos.second.at(PossibleParameter::vel_x) / x_scaling),                                                                 // Velocity
         vehicle_index                                                                                                                     // ID
      };

      if (on_straight_section >= 0) {
         r->findSectionWithID(on_straight_section)->getMyRoad().addOther(veh);
      }
      else if (traversion_from >= 0 && traversion_to >= 0) {
         const auto from_section = r->findSectionWithID(traversion_from);
         if (from_section) {
            from_section->addNonegoOnCrossingTowards(traversion_to, veh);
         }
         else {
            addError("Cannot place car on connection from sec '" + std::to_string(traversion_from) + "' to sec '" + std::to_string(traversion_to) + "' because the origin section does not exist.");
         }
      }
      else {
         addError("Car '" + vehicle_name + "' is placed neither on a straight section nor on a junction.");
      }
   }
}

void LiveSimGenerator::generate(
   const std::string& base_output_name,
   const std::set<int>& agents_to_draw_arrows_for,
   const std::string& stage_name,
   const MCTrace& trace,
   const LiveSimType visu_type,
   const std::vector<vfm::OutputType> single_images_output_types,
   const double x_scaling,
   const double time_factor,
   const long sleep_for_ms)
{
   std::shared_ptr<RoadGraph> road_graph{ getRoadGraphTopologyFrom(trace) };

   std::string image_file_output = base_output_name + ".png";
   std::filesystem::path morty_progress_path{ base_output_name };
   morty_progress_path = morty_progress_path.parent_path().parent_path().parent_path();
   morty_progress_path /= "progress.morty";

   auto& vehicle_names_without_ego = m_trajectory_provider.getVehicleNames(true);

   Env2D env{ vehicle_names_without_ego.size() };

   GifRecorder gif_recorder(base_output_name + ".gif", visu_type & LiveSimType::gif_animation);

   auto& ego_trajectory = m_trajectory_provider.getEgoTrajectory();
   int trajectory_length = ego_trajectory.size();

   for (size_t trajectory_index = 0; trajectory_index < trajectory_length; trajectory_index++) {
      std::cout << StaticHelper::printProgress("Rendering Progress", trajectory_index, trajectory_length, 30); // Print output (override itself)
      StaticHelper::writeTextToFile(std::to_string(trajectory_index) + "#" + std::to_string(trajectory_length - 1) + "#" + stage_name, morty_progress_path.string());

      equipRoadGraphWithCars(road_graph, trajectory_index, x_scaling);

      ExtraVehicleArgs extra_var_vals = {}; // Extra data currently only used to inform about turn signals
      
      const auto& current_ego = ego_trajectory[trajectory_index];

      // Provide ego data
      if (current_ego.second.at(PossibleParameter::turn_signal_left) > 0.5)
         extra_var_vals.insert({ "ego.turn_signals", "LEFT" });
      if (current_ego.second.at(PossibleParameter::turn_signal_right) > 0.5)
         extra_var_vals.insert({ "ego.turn_signals", "RIGHT" });

      extra_var_vals.insert({ "ego.gaps___609___.i_agent_front", std::to_string(current_ego.second.at(PossibleParameter::gap_0_i_agent_front))});
      extra_var_vals.insert({ "ego.gaps___619___.i_agent_front", std::to_string(current_ego.second.at(PossibleParameter::gap_1_i_agent_front))});
      extra_var_vals.insert({ "ego.gaps___629___.i_agent_front", std::to_string(current_ego.second.at(PossibleParameter::gap_2_i_agent_front))});
      extra_var_vals.insert({ "ego.gaps___609___.i_agent_rear", std::to_string(current_ego.second.at(PossibleParameter::gap_0_i_agent_rear))});
      //extra_var_vals.insert({ "ego.gaps___619___.i_agent_rear", std::to_string(current_ego.second.at(PossibleParameter::gap_1_i_agent_rear))});
      //extra_var_vals.insert({ "ego.gaps___629___.i_agent_rear", std::to_string(current_ego.second.at(PossibleParameter::gap_2_i_agent_rear))});
      // TODO: Above 2 currently not used.

      // Provide data for other vehicles
      for (const auto& vehicle_name : vehicle_names_without_ego)
      {
         auto& traj_pos = m_trajectory_provider.getVehicleTrajectory(vehicle_name)[trajectory_index];

         if (traj_pos.second.at(PossibleParameter::turn_signal_left) > 0.5)
            extra_var_vals.insert({ vehicle_name + ".turn_signals", "LEFT" });
         if (traj_pos.second.at(PossibleParameter::turn_signal_right) > 0.5)
            extra_var_vals.insert({ vehicle_name + ".turn_signals", "RIGHT" });
      }

      if (visu_type & LiveSimType::incremental_image_output)
      {
         image_file_output = base_output_name + "_" + std::to_string(trajectory_index);
      }

      // Add frame to gif
      auto img = updateOutputImages(
         env, 
         visu_type, 
         single_images_output_types, 
         image_file_output, 
         extra_var_vals, 
         m_trajectory_provider.getDataTrace().at(trajectory_index),
         m_trajectory_provider.getDataTrace().size() > trajectory_index + 1 ? m_trajectory_provider.getDataTrace().at(trajectory_index + 1) : nullptr,
         VARIABLES_TO_BE_PAINTED,
         agents_to_draw_arrows_for,
         road_graph);

      // Determine frame duration from trajectory
      double frame_duration;
      if (trajectory_index == trajectory_length - 1)
         frame_duration = current_ego.first - ego_trajectory[trajectory_index - 1l].first;
      else
         frame_duration = ego_trajectory[trajectory_index + 1l].first - current_ego.first;

      gif_recorder.addImage(img, frame_duration * time_factor * 100); // x 100 because apparently the gif library takes.. centiseconds?!

      // Only live image
      if ((visu_type & LiveSimType::constant_image_output) && !(visu_type & LiveSimType::incremental_image_output)) {
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
   }

	std::cout << std::endl;
   StaticHelper::removeFileSafe(morty_progress_path);

   gif_recorder.finish();
}

void LiveSimGenerator::paintVarBox(Image& img, DataPack& data, std::vector<PainterVariableDescription>& variables_to_be_painted)
{
   // TODO: Note that, technically, the var_box is painted in the wrong coordinate system, which is just the plain
   // image's coordinate system, as opposed to the model checker's coordinate system. Since the 3D view is not yet
   // fully implemented in this regard, i.e., uses an identity function as translator, this actually works out well:
   // - in 3D or the combined view, the var box is painted at the desired (untranslated) position.
   // - in 2D, the var box is painted beyond the image area due to translation. This is, however, not so bad since it 
   // doesn't fit in this case, anyway. Has to be fixed eventually, though.

   std::vector<PainterButtonDescription> button_descriptions{};
   bool first{ true };
   Image background_image{};
   Image sw_chunk{};
   int sw_chunk_x{};
   int sw_chunk_y{};
   int i{ 0 };
   
   img.autoExpandToTheBottom(1); // Expanding minimally by one (lowest possible, possibly slowest) means that the image grows not faster than it must.
   defaultPaintFunction(data, img, variables_to_be_painted, button_descriptions, std::string(""), first, background_image, sw_chunk, sw_chunk_x, sw_chunk_y, i);
   img.deactivateAutoExpandToTheBottom();
}

std::shared_ptr<Image> LiveSimGenerator::updateOutputImages(
   Env2D& env, 
   LiveSimType visu_type, 
   const std::vector<vfm::OutputType> single_images_output_types, 
   const std::string image_file_output,
   ExtraVehicleArgs var_vals, 
   const DataPackPtr data,
   const DataPackPtr future_data,
   const std::shared_ptr<std::vector<PainterVariableDescription>> variables_to_be_painted,
   const std::set<int>& agents_to_draw_arrows_for,
   const std::shared_ptr<RoadGraph> road_graph
   )
{
   const bool activate_pdf{ ((visu_type & LiveSimType::constant_image_output) || (visu_type & LiveSimType::incremental_image_output))
      && StaticHelper::count<OutputType>(single_images_output_types, OutputType::pdf)};

   bool CREATE_COCKPIT_VIEW = visu_type & LiveSimType::cockpit;
   bool CREATE_BIRDSEYE_VIEW = visu_type & LiveSimType::birdseye;

   int width_cpv = 1000;
   int height_cpv = 300;

   auto actual_future_data = ((visu_type & LiveSimType::constant_image_output) || (visu_type & LiveSimType::incremental_image_output) || (visu_type & LiveSimType::always_paint_arrows))
      ? future_data
      : nullptr;

   auto birds_eye = CREATE_BIRDSEYE_VIEW 
      ? env.getBirdseyeView(
         agents_to_draw_arrows_for,
         var_vals, 
         actual_future_data, 
         activate_pdf,
         CREATE_COCKPIT_VIEW ? 0 : 900, // TODO: Not so nice to hard-code this. But cropping the birds-eye view like this is often beneficial when used as a figure in a text.
         CREATE_COCKPIT_VIEW ? 0 : 700,
         road_graph)
      : nullptr;

   if (birds_eye)
   {
      width_cpv = birds_eye->getWidth();
      height_cpv = width_cpv / 5;
   }

   auto cockpit = CREATE_COCKPIT_VIEW
      ? env.getCockpitView(
         agents_to_draw_arrows_for,
         var_vals,
         actual_future_data,
         activate_pdf,
         road_graph,
         width_cpv, 
         height_cpv)
      : nullptr;
   std::shared_ptr<Image> img;

   if (birds_eye && cockpit) {
      img = std::make_shared<Image>(std::max(birds_eye->getWidth(), cockpit->getWidth()), birds_eye->getHeight() + cockpit->getHeight());

      if (activate_pdf) {
         img->restartPDF();
      }

      img->fillImg(WHITE);
      img->insertImage(0, 0, *cockpit, false);
      img->insertImage(0, cockpit->getHeight(), *birds_eye, false);
   }
   else if (birds_eye) {
      img = birds_eye;
   }
   else if (cockpit) {
      img = cockpit;
   }

   if (((visu_type & LiveSimType::constant_image_output) || (visu_type & LiveSimType::incremental_image_output)) && img)
   {
      if (data && CREATE_BIRDSEYE_VIEW && CREATE_COCKPIT_VIEW) {
         auto old_pdf{ img->getPdfDocument() };
         img->setPdfDocument(nullptr); // Cut out paint box from PDF. TODO: Make painting of mirror nice instead of leaving it out.

         std::vector<PainterVariableDescription> actual_variables_to_be_painted{};

         for (const auto& var_regex : *variables_to_be_painted) {
            if (StaticHelper::stringStartsWith(var_regex.first, REGEX_PREFIX)) {
               std::set<std::string> matching_vars{ data->getAllVariableNamesMatchingRegex(var_regex.first.substr(REGEX_PREFIX.size())) };

               if (matching_vars.empty()) { // Can only occur if regex matched nothing.
                  matching_vars.insert(var_regex.first.substr(REGEX_PREFIX.size())); // Insert the ill-formed regex in that case to display with "-".
               }

               for (const auto& var_name : matching_vars) {
                  actual_variables_to_be_painted.push_back({ var_name, var_regex.second });
               }
            }
            else {
               actual_variables_to_be_painted.push_back(var_regex);
            }
         }

         *variables_to_be_painted = actual_variables_to_be_painted;

         paintVarBox(*img, *data, *variables_to_be_painted);
         img->setPdfDocument(old_pdf);
      }

      for (const auto& single_images_output_type : single_images_output_types) {
         if (single_images_output_type != OutputType::pdf || activate_pdf) {
            auto path{ image_file_output };

            if (single_images_output_type == OutputType::pdf) {
               path = StaticHelper::removeLastFileExtension(image_file_output, "/") + "/pdf/";
               StaticHelper::createDirectoriesSafe(path);
               path += StaticHelper::getLastFileExtension(image_file_output, "/");
            }

            img->store(path, single_images_output_type);
         }
      }
   }

   return img;
}

void LiveSimGenerator::GifRecorder::addImage(std::shared_ptr<Image> img, double frame_duration)
{
   if (!m_active)
      return;

   if (!m_is_initialized)
   {
      GifBegin(&m_gif_writer, m_name.c_str(), img->getWidth(), img->getHeight(), frame_duration);

      m_is_initialized = true;
   }

   GifWriteFrame(&m_gif_writer, (uint8_t*)img->getRawImage().data(), std::min(img->getWidth(), 4000), std::min(img->getHeight(), 4000), frame_duration);
}

void LiveSimGenerator::GifRecorder::finish()
{
   if (!m_active)
      return;

   GifEnd(&m_gif_writer);
}
