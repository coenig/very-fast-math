//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#pragma once
#include "highway_translators.h"
#include "geometry/images.h"
#include "road_graph.h"

namespace vfm {

class HighwayImage : public Image, public Failable
{
public:
   constexpr static int EGO_MOCK_ID{ -100 };

   HighwayImage(const int width, const int height, const std::shared_ptr<HighwayTranslator> translator, const int num_lanes);
   HighwayImage(const std::shared_ptr<HighwayTranslator> translator, const int num_lanes);

   void setupVPointFor3DPerspective(const int num_lanes, const Vec2D& dim); // 3D-Specific - TODO: Needs to go away.
   void paintEarthAndSky(const Vec2D& dim);                                 // 3D-Specific - TODO: Needs to go away.

   void setTranslator(const std::shared_ptr<VisTranslator> function) override;

   void plotCar2D(const float thick, const Vec2Df& pos_ego, const Color& fill_color, const Color& car_frame_color);
   void plotCar3D(const Vec2Df& pos_ego, const Color& fill_color, const Color& car_frame_color);

   void doShoulder(
      const bool min_lane,
      const bool is_current_shoulder,
      const bool is_last_shoulder,
      const bool is_last_last_shoulder,
      const Vec2D& last_one,
      const Vec2D& new_one,
      const bool is_extending,
      const float actual_arc_length);

   void removeNonExistentLanesAndMarkShoulders(
      const StraightRoadSection& lane_structure,
      const std::shared_ptr<CarPars> ego,
      const Vec2D& tl_orig,
      const Vec2D& br_orig,
      const bool infinite_road, 
      const Vec2D& dim,
      std::vector<ConnectorPolygonEnding>& connections);

   void setPerspective(
      const float street_height,
      const float num_lanes,
      const float ego_offset_x,
      const int min_lane,
      const int max_lane,
      const float street_top,
      const float ego_car_lane, 
      const Vec2D& dim);

   void paintStraightRoadSceneFromData(
      StraightRoadSection& lane_structure, 
      const DataPack& data, 
      const std::shared_ptr<DataPack> future_data);

   void paintStraightRoadSceneSimple(
      const CarPars& ego,
      const CarParsVec& others,
      const std::map<int, std::pair<float, float>>& future_positions_of_others,
      const float ego_offset_x = 0,
      const std::map<std::string, std::string>& var_vals = {},
      const bool print_agent_ids = true);

   /// Core function for painting a straight road section.
   std::vector<ConnectorPolygonEnding> paintStraightRoadScene(
      StraightRoadSection& lane_structure,
      const bool infinite_road,
      const float ego_offset_x = 0,
      const std::map<std::string, std::string>& var_vals = {},
      const bool print_agent_ids = true,
      const Vec2D& dim = { 500, 100 });

   void paintRoadGraph(
      const std::shared_ptr<RoadGraph> r,
      const Vec2D& dim,
      const float ego_offset_x = 0,
      const std::map<std::string, std::string>& var_vals = {},
      const bool print_agent_ids = true,
      const float TRANSLATE_X = 60,
      const float TRANSLATE_Y = 20);

   std::shared_ptr<HighwayTranslator> getHighwayTranslator() const;

private:
   std::shared_ptr<HighwayTranslator> highway_translator_{};
   std::shared_ptr<Plain2DTranslator> plain_2d_translator_{ std::make_shared<Plain2DTranslator>() };
   float cnt_{ -150 };
   float step_{ 0.05 };
   int num_lanes_{}; // TODO: Needed only for setting up 3D perspective, which should go into the 3D part.
};
} // vfm