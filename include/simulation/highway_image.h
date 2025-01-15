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
   HighwayImage(const int width, const int height, const std::shared_ptr<HighwayTranslator> translator, const int num_lanes);
   HighwayImage(const std::shared_ptr<HighwayTranslator> translator, const int num_lanes);

   void setupVPointFor3DPerspective(const int num_lanes); // 3D-Specific - TODO: Needs to go away.
   void paintEarthAndSky();                               // 3D-Specific - TODO: Needs to go away.

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
      const CarPars& ego,
      const Vec2D& tl_orig,
      const Vec2D& br_orig);

   void setPerspective(
      const float street_height,
      const float num_lanes,
      const float ego_offset_x,
      const int min_lane,
      const int max_lane,
      const float street_top,
      const float ego_car_lane);

   void paintHighwaySceneFromData(StraightRoadSection& lane_structure, const DataPack& data, const std::shared_ptr<DataPack> future_data);

   void paintHighwayScene(
      const CarPars& ego,
      const CarParsVec& others,
      const std::map<int, std::pair<float, float>>& future_positions_of_others,
      const float ego_offset_x = 0,
      const std::map<std::string, std::string>& var_vals = {},
      const bool print_agent_ids = false);

   /// Core function for painting a straight road section.
   void paintHighwayScene(
      StraightRoadSection& lane_structure,
      const CarPars& ego,
      const CarParsVec& others,
      const std::map<int, std::pair<float, float>>& future_positions_of_others,
      const float ego_offset_x = 0,
      const std::map<std::string, std::string>& var_vals = {},
      const bool print_agent_ids = false);

   std::shared_ptr<HighwayTranslator> getHighwayTranslator() const;

private:
   float offset_dashed_lines_on_highway_ = 0;
   std::shared_ptr<HighwayTranslator> highway_translator_{};
   Plain2DTranslator plain_2d_translator_{};
   float cnt_{ -150 };
   float step_{ 0.05 };
};
} // vfm