//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/cex_processing/mc_visualization_launchers.h"
#include <thread>


using namespace vfm;
using namespace mc;
using namespace trajectory_generator;

std::string VisualizationLaunchers::writeProseSingleStep(
   const CarParsVec& others_past_vec,
   const CarParsVec& others_current_vec,
   const CarParsVec& others_future_vec,
   const CarPars& past_ego,
   const CarPars& ego,
   const int seconds)
{
   const auto LANE_CHANGE{ [](const float past_lane, const float current_lane) {
      if (past_lane > current_lane) {
         if (StaticHelper::isFloatInteger(past_lane)) {
            return " starts a lane change to the left";
         }
         else {
            return " finshes its lane change to the left";
         }
      }
      else if (past_lane < current_lane) {
         if (StaticHelper::isFloatInteger(past_lane)) {
            return +" starts a lane change to the right";
         }
         else {
            return +" finshes its lane change to the right";
         }
      }
   } };

   const auto LANE_INFO{ [](const float other_lane, const float ego_lane) -> std::string {
      if (ego_lane - other_lane > 0) {
         return StaticHelper::floatToStringNoTrailingZeros(truncf((ego_lane - other_lane) * 10) / 10) + " lane" + ((int)(ego_lane - other_lane) != 1 ? "s" : "") + " right of EGO";
      }
      else if (ego_lane - other_lane < 0) {
         return StaticHelper::floatToStringNoTrailingZeros(truncf((other_lane - ego_lane) * 10) / 10) + " lane" + ((int)(other_lane - ego_lane) != 1 ? "s" : "") + " left of EGO";
      }
      else {
         return " on EGO's lane";
      }
   } };

   const auto LONG_INFO{ [](const float other_rel_pos, const float ego_lane, const float other_lane) -> std::string {
      if (std::abs(ego_lane - other_lane) <= 0.5 && std::abs(other_rel_pos) <= 5) { // TODO: Get car length in a good way.
         std::string side{ std::abs(ego_lane - other_lane) == 0 ? "" : (std::abs(ego_lane - other_lane) > 0 ? "/left side" : "/right side")};
         return "collides with EGO's " + std::string(other_rel_pos < 0 ? "rear" : "front") + side;
      }

      std::string addon{};
      if (std::abs(other_rel_pos) > 65) {
         addon = "way ";
      }
      else if (std::abs(other_rel_pos) > 0 && std::abs(other_rel_pos) < 10) {
         addon = "close ";
      }

      if (other_rel_pos == 0) {
         return "next to EGO";
      }
      else if (other_rel_pos < 0) {
         return addon + "behind EGO";
      }
      else {
         return addon + "in front of EGO";
      }
   } };

   const bool first{ others_past_vec.empty() };
   const bool last{ others_future_vec.empty() };
   std::vector<std::string> vec{};
   std::string ego_str{};

   if (first || last) {
      ego_str = "EGO is on lane " + StaticHelper::floatToStringNoTrailingZeros(truncf((ego.car_lane_) * 10) / 10) + " and drives with " + std::to_string(ego.car_velocity_) + "m/s";
   }
   else {
      if (past_ego.car_velocity_ - 5 > ego.car_velocity_) {
         ego_str += std::string(ego_str.empty() ? "EGO" : " and") + " brakes hard";
      }
      //else if (past_ego.car_velocity_ > ego.car_velocity_) {
      //   ego_str += std::string(ego_str.empty() ? "EGO" : " and") + " brakes";
      //}
      //else if (past_ego.car_velocity_ < ego.car_velocity_) {
      //   ego_str += std::string(vec[i].empty() ? "EGO" : " and") + " accelerates";
      //}
      if (past_ego.car_lane_ != ego.car_lane_) {
         ego_str += std::string(ego_str.empty() ? "EGO" : " and") + LANE_CHANGE(past_ego.car_lane_, ego.car_lane_);
      }
   }

   for (int i = 0; i < others_current_vec.size(); i++) {
      vec.insert(vec.begin() + i, "");

      if (first || last) {
         vec[i] += "car" + std::to_string(i) + " is " + LANE_INFO(ego.car_lane_, others_current_vec[i].car_lane_) + " and " + LONG_INFO(others_current_vec[i].car_rel_pos_, ego.car_lane_, others_current_vec[i].car_lane_);
      }
      else {
         if (others_past_vec.at(i).car_rel_pos_ <= others_current_vec[i].car_rel_pos_ && others_current_vec[i].car_rel_pos_ > others_future_vec.at(i).car_rel_pos_) {
            vec[i] += std::string(vec[i].empty() ? "car" + std::to_string(i) : " and") + " starts to brake";
         }
         else if (others_past_vec.at(i).car_rel_pos_ >= others_current_vec[i].car_rel_pos_ && others_current_vec[i].car_rel_pos_ < others_future_vec.at(i).car_rel_pos_) {
            vec[i] += std::string(vec[i].empty() ? "car" + std::to_string(i) : " and") + " starts to accelerate";
         }
         if (others_past_vec.at(i).car_lane_ != others_current_vec[i].car_lane_) {
            vec[i] += std::string(vec[i].empty() ? "car" + std::to_string(i) : " and") + LANE_CHANGE(others_past_vec.at(i).car_lane_, others_current_vec[i].car_lane_);
         }

         if (others_current_vec[i].car_rel_pos_ <= 0
            && others_future_vec.at(i).car_rel_pos_ > 0) {
            vec[i] += std::string(vec[i].empty() ? "car" + std::to_string(i) : " and") + " overtakes EGO";
         }
         else if (others_current_vec[i].car_rel_pos_ > 0
            && others_future_vec.at(i).car_rel_pos_ <= 0) {
            vec[i] += std::string(vec[i].empty() ? "car" + std::to_string(i) : " and") + " is overtaken by EGO";
         }

         for (int j = 0; j < others_current_vec.size(); j++) {
            if (j != i) {
               if (others_current_vec[i].car_rel_pos_ <= others_current_vec[j].car_rel_pos_
                  && others_future_vec.at(i).car_rel_pos_ > others_future_vec.at(j).car_rel_pos_) {
                  vec[i] += std::string(vec[i].empty() ? "car" + std::to_string(i) : " and") + " overtakes car" + std::to_string(j);
               }
            }
         }
      }
   }

   bool empty{ ego_str.empty() };

   for (const auto& s : vec) {
      if (!s.empty()) empty = false;
   }

   if (!empty) {
      std::string res{ ego_str };

      for (const auto& s : vec) {
         res += (res.empty() ? "" : (s.empty() ? "" : ", ")) + s;
      }

      if (first) {
         res = "In the beginning, " + res + ". ";
      }
      else if (last) {
         res = "In the end, " + res + ". ";
      }
      else {
         res = "After " + std::to_string(seconds) + " second" + (seconds > 1 ? "s" : "") + ", " + res + ". ";
      }

      return res;
   }

   return "";
}

inline std::pair<std::vector<CarPars>, std::vector<CarParsVec>> createOthersVecs(const DataPackTrace data_trace, const StraightRoadSection lane_structure)
{
   const float LANE_CONSTANT{ ((float)lane_structure.getNumLanes() - 1) * 2 };
   int num_cars{};
   std::vector<CarPars> ego_values{};
   std::vector<CarParsVec> others_values{};

   for (; data_trace.at(0)->isDeclared("veh___6" + std::to_string(num_cars) + "9___.rel_pos"); num_cars++);

   for (const auto& data : data_trace) {
      CarPars ego{};
      CarParsVec others{};

      ego.car_lane_ = (LANE_CONSTANT - data->getSingleVal("ego.on_lane")) / 2;
      ego.car_rel_pos_ = data->getSingleVal("ego.abs_pos");
      ego.car_velocity_ = data->getSingleVal("ego.v");

      for (int i{ 0 }; i < num_cars; i++) {
         CarPars other{};

         other.car_id_ = i;
         other.car_lane_ = (LANE_CONSTANT - data->getSingleVal("veh___6" + std::to_string(i) + "9___.on_lane")) / 2;
         other.car_rel_pos_ = data->getSingleVal("veh___6" + std::to_string(i) + "9___.rel_pos");
         other.car_velocity_ = data->getSingleVal("veh___6" + std::to_string(i) + "9___.v");

         others.push_back(other);
      }

      others_values.push_back(others);
      ego_values.push_back(ego);
   }

   return { ego_values, others_values };
}

std::string VisualizationLaunchers::writeProseTrafficScene(const MCTrace trace)
{
   const auto road_graph{ getLaneStructureFrom(trace) };
   const auto config = InterpretationConfiguration::getLaneChangeConfiguration();
   const MCinterpretedTrace interpreted_trace(0, { trace }, config);
   const auto data_vec{ interpreted_trace.getDataTrace() };
   const auto pair{ createOthersVecs(data_vec, road_graph->getMyRoad()) };
   const auto ego_vec{ pair.first };
   const auto others_vec{ pair.second };
   std::string res{};

   for (int i = 0; i < data_vec.size(); i++) {
      CarParsVec past_vec{ i - 1 >= 0 ? others_vec.at(i - 1) : CarParsVec{} };
      CarPars past_ego{ i - 1 >= 0 ? ego_vec.at(i - 1) : CarPars{} };
      CarParsVec current_vec{ others_vec.at(i) };
      CarPars current_ego{ ego_vec.at(i) };
      CarParsVec future_vec{ i + 1 < others_vec.size() ? others_vec.at(i + 1) : CarParsVec{} };
      res += writeProseSingleStep(past_vec, current_vec, future_vec, past_ego, current_ego, i);
   }

   return res;
}

bool VisualizationLaunchers::quickGenerateGIFs(
   const std::set<int>& cex_nums_to_generate,
   const std::string& path_cropped,
   const std::string& file_name_without_txt_extension,
   const CexType& cex_type,
   const bool export_basic,
   const bool export_with_smooth_arrows,
   const bool export_without_smooth_arrows,
   const std::set<int>& agents_to_draw_arrows_for) 
{
   if (cex_type != CexTypeEnum::smv && cex_type != CexTypeEnum::kratos && cex_type != CexTypeEnum::msatic) {
      Failable::getSingleton()->addError("Cex type '" + cex_type.getEnumAsString() + "' not allowed. CEX generation aborted.");
      return false;
   }

   const std::string path_base{ path_cropped + "/" };

   auto traces{ cex_type == CexTypeEnum::msatic
      ? StaticHelper::extractMCTracesFromMSATICFile(path_base + file_name_without_txt_extension + ".txt")
      : (cex_type == CexTypeEnum::smv
         ? StaticHelper::extractMCTracesFromNusmvFile(path_base + file_name_without_txt_extension + ".txt")
         : (StaticHelper::extractMCTracesFromKratosFile(path_base + file_name_without_txt_extension + ".txt"))) };

   if (traces.empty()) {
      Failable::getSingleton()->addNote("Received empty list of CEXs in file '" + path_base + file_name_without_txt_extension + ".txt" + "'. Nothing to do.");
      return false;
   }

   for (int index : cex_nums_to_generate) {
      auto& trace = traces.at(index);
      const std::string path{ path_base + std::to_string(index) + "/"};

      if (trace.empty()) {
         Failable::getSingleton()->addNote("Received empty CEX at index '" + std::to_string(index) + "' in file '" + path + file_name_without_txt_extension + ".txt" + "'. Nothing to do.");
         continue;
      }

      StaticHelper::createDirectoriesSafe(path);
      StaticHelper::writeTextToFile(StaticHelper::serializeMCTraceNusmvStyle(trace), path + file_name_without_txt_extension + "_unscaled.smv");

      Failable::getSingleton()->addNote("Applying scaling.");
      ScaleDescription::createTimescalingFile(path_base);
      auto ts_description{ ScaleDescription(StaticHelper::readFile(path_base + TIMESCALING_FILENAME)) };
      StaticHelper::applyTimescaling(trace, ts_description);
      StaticHelper::writeTextToFile(StaticHelper::serializeMCTraceNusmvStyle(trace), path + file_name_without_txt_extension + ".smv");

      StaticHelper::writeTextToFile(writeProseTrafficScene(trace), path + "prose_scenario_description.txt");

      std::string trial_name{ file_name_without_txt_extension };
      std::string comments_file{ path_cropped + "/" + "Comments.csv" };
      StaticHelper::writeTextToFile(path + trial_name + ";\n", comments_file, true);

      VisualizationScales gen_config_non_smooth{};
      gen_config_non_smooth.x_scaling = 1;
      gen_config_non_smooth.duration_scale = 1;
      gen_config_non_smooth.frames_per_second_gif = 0;
      gen_config_non_smooth.frames_per_second_osc = 0;
      gen_config_non_smooth.gif_duration_scale = 1 / ts_description.getTimeScalingFactor();

      // Smoothing by adding interpolated frames
      auto gen_config_smooth = VisualizationScales{ gen_config_non_smooth };
      gen_config_smooth.frames_per_second_gif = 40;
      gen_config_smooth.frames_per_second_osc = 40;

      //VisualizationLaunchers::interpretAndGenerate(
      //   trace,
      //   path,
      //   "preview",
      //   SIM_TYPE_SMOOTH_WITH_ARROWS_BIRDSEYE_ONLY,
      //   agents_to_draw_arrows_for,
      //   gen_config_smooth, "preview");

      VisualizationLaunchers::interpretAndGenerate(
         trace,
         path,
         "preview2",
         SIM_TYPE_REGULAR_BIRDSEYE_ONLY_NO_GIF,
         agents_to_draw_arrows_for,
         gen_config_non_smooth, "preview 2");

      if (export_basic) {
         VisualizationLaunchers::interpretAndGenerate(
            trace,
            path,
            "cex-full",
            SIM_TYPE_REGULAR,
            agents_to_draw_arrows_for,
            gen_config_non_smooth, "full (1/7)");

         VisualizationLaunchers::interpretAndGenerate(
            trace,
            path,
            "cex-birdseye",
            SIM_TYPE_REGULAR_BIRDSEYE_ONLY,
            agents_to_draw_arrows_for,
            gen_config_non_smooth, "birdseye (2/7)");

         VisualizationLaunchers::interpretAndGenerate(
            trace,
            path,
            "cex-cockpit-only",
            static_cast<LiveSimGenerator::LiveSimType>(LiveSimGenerator::LiveSimType::gif_animation | LiveSimGenerator::LiveSimType::cockpit | LiveSimGenerator::LiveSimType::incremental_image_output),
            agents_to_draw_arrows_for,
            gen_config_non_smooth, "cockpit (3/7)");
      }

      if (export_with_smooth_arrows) {
         VisualizationLaunchers::interpretAndGenerate(
            trace,
            path,
            "cex-smooth-with-arrows-full",
            SIM_TYPE_SMOOTH_WITH_ARROWS,
            agents_to_draw_arrows_for,
            gen_config_smooth, "smooth-with-arrows (6/7)");

         VisualizationLaunchers::interpretAndGenerate(
            trace,
            path,
            "cex-smooth-with-arrows-birdseye",
            SIM_TYPE_SMOOTH_WITH_ARROWS_BIRDSEYE_ONLY,
            agents_to_draw_arrows_for,
            gen_config_smooth, "smooth-with-arrows-birdseye (7/7)");
      }

      if (export_without_smooth_arrows) {
         VisualizationLaunchers::interpretAndGenerate(
            trace,
            path,
            "cex-smooth-full",
            SIM_TYPE_SMOOTH,
            agents_to_draw_arrows_for,
            gen_config_smooth, "full-smooth (4/7)");

         VisualizationLaunchers::interpretAndGenerate(
            trace,
            path,
            "cex-smooth-birdseye",
            SIM_TYPE_SMOOTH_BIRDSEYE_ONLY,
            agents_to_draw_arrows_for,
            gen_config_smooth, "birdseye-smooth (5/7)");
      }
   }

   return true;
}

std::shared_ptr<RoadGraph> vfm::mc::trajectory_generator::VisualizationLaunchers::getLaneStructureFrom(const MCTrace& trace)
{
   if (trace.empty()) Failable::getSingleton()->addError("Received empty trace for 'getLaneStructureFrom'.");

   const auto first_state{ trace.at(0).second };

   const auto segment_begin_name = [](const int sec, const int seg) -> std::string { return "section_" + std::to_string(sec) + "_segment_" + std::to_string(seg) + "_pos_begin"; };
   const auto segment_min_lane_name = [](const int sec, const int seg) -> std::string { return "section_" + std::to_string(sec) + "_segment_" + std::to_string(seg) + "_min_lane"; };
   const auto segment_max_lane_name = [](const int sec, const int seg) -> std::string { return "section_" + std::to_string(sec) + "_segment_" + std::to_string(seg) + "_max_lane"; };
   const auto connection = [](const int sec, const int con) -> std::string { return "env.outgoing_connection_" + std::to_string(con) + "_of_section_" + std::to_string(sec); };

   std::vector<std::shared_ptr<RoadGraph>> road_graphs{};

   for (int sec = 0; first_state.count(segment_begin_name(sec, 0)); sec++) {
      road_graphs.push_back(std::make_shared<RoadGraph>(sec));
      StraightRoadSection lane_structure{std::stoi(first_state.at("num_lanes")), std::stof(first_state.at("section_" + std::to_string(sec) + "_end")) };
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
         std::stof(first_state.at("section_" + std::to_string(sec) + ".source.y"))});
      
      road_graphs[sec]->setAngle(2.0 * 3.1415 * std::stof(first_state.at("section_" + std::to_string(sec) + ".angle")) / 360.0);
      road_graphs[sec]->setMyRoad(lane_structure);
   }

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

bool vfm::mc::trajectory_generator::VisualizationLaunchers::interpretAndGenerate(
   const MCTrace& trace, 
   const std::string& out_pathname_raw, 
   const std::string& out_filename_raw, 
   const LiveSimGenerator::LiveSimType sim_type,
   const std::set<int>& agents_to_draw_arrows_for,
   const VisualizationScales& settings,
   const std::string& stage_name,
   const bool generate_osc,
   const bool generate_gif)
{
   const std::string full_path = out_pathname_raw + "/" + out_filename_raw;
   std::filesystem::create_directories(full_path);
   std::string final_name = out_filename_raw;
   if (settings.duration_scale != 1.0)
      final_name += "_duration_scale_" + toLimStr(settings.duration_scale);
   if (settings.x_scaling != 1.0)
      final_name += "_x_scale_" + toLimStr(settings.x_scaling);

   std::string out_path = full_path + "/" + final_name;

   const auto config = InterpretationConfiguration::getLaneChangeConfiguration();
   MCinterpretedTrace interpreted_trace(0, { trace }, config);

   assert(interpreted_trace.getEgoTrajectory().size() == interpreted_trace.getDataTrace().size());

   interpreted_trace.applyScaling(settings.x_scaling, 1.0, settings.duration_scale);
   interpreted_trace.printDebug("FINAL");

   if (generate_osc)
   {
      MCinterpretedTrace interpreted_trace_osc = interpreted_trace.clone();
      interpreted_trace_osc.applyInterpolation(settings.duration_scale * config.m_default_step_time * settings.frames_per_second_osc);

      std::string header =
         "# Imports config file and scenario file"
         "\nimport \"StraightThreeLaneHighwayRQ36.osm\""
         "\nimport \"../../../vehicle/system_cuts.osc\"";

      std::string osc_without_ego = OSCgenerator(interpreted_trace_osc).generate(final_name, false, header);
      std::string osc_with_ego = OSCgenerator(interpreted_trace_osc).generate(final_name, true, header);
      std::string csv_with_ego = OSCgenerator(interpreted_trace_osc).generate_as_csv();

      std::cout << "OSC WITH EGO: \n" << osc_with_ego << std::endl;

      std::ofstream file_without_ego(out_path + "_free_ego.osc");
      file_without_ego << osc_without_ego;
      file_without_ego.close();

      std::ofstream file_with_ego(out_path + ".osc");
      file_with_ego << osc_with_ego;
      file_with_ego.close();

      std::ofstream csv_file_with_ego(out_path + ".csv");
      csv_file_with_ego << csv_with_ego;
      csv_file_with_ego.close();
   }
   if (generate_gif)
   {
      interpreted_trace.applyInterpolation(settings.duration_scale * config.m_default_step_time * settings.frames_per_second_gif * settings.gif_duration_scale);

      assert(interpreted_trace.getEgoTrajectory().size() == interpreted_trace.getDataTrace().size());

      interpreted_trace.applyScaling(1.0, -1.0, 1.0); // flip y

      auto road_graph{ getLaneStructureFrom(trace) };

      // TODO
      auto others = createOthersVecs(interpreted_trace.getDataTrace(), road_graph->getMyRoad());
      road_graph->getMyRoad().setEgo(std::make_shared<CarPars>(others.first.at(0)));
      road_graph->getMyRoad().setOthers(others.second.at(0));
      
      LiveSimGenerator(interpreted_trace).generate(out_path,
         agents_to_draw_arrows_for,
         road_graph,
         stage_name,
         sim_type,
         { OutputType::png, OutputType::pdf },
         settings.x_scaling,
         settings.gif_duration_scale,
         0);
   }

   return true;
}

extern "C"
bool processCEX(
   const char* path_cropped,
   const char* cex_type,
   const bool include_smooth,
   const bool with_smooth_arrows)
{
   bool success{ vfm::mc::trajectory_generator::VisualizationLaunchers::quickGenerateGIFs(
      { 0 }, // For now, always generate only the first CEX.
      std::string(path_cropped),
      "debug_trace_array",
      CexType(std::string(cex_type)),
      include_smooth,
      true, // without_smooth_arrows
      with_smooth_arrows) };

   if (!success) {
      Failable::getSingleton()->addError("CEX generation failed.");
   }

   return success;
}
