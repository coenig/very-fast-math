//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "failable.h"
#include "geometry/polygon_2d.h"
#include "geometry/images.h"
#include "simulation/highway_image.h"
#include <memory>


using namespace vfm::fsm;

namespace vfm {

static constexpr int RIGHT_MARGIN = 300;
static constexpr int LEFT_MARGIN = -300;
static constexpr float STREET_LENGTH_FOR_IMG_DIM = 12;
static constexpr float RELATION_FOR_IMG_DIM = (RIGHT_MARGIN - LEFT_MARGIN) / 45;
static constexpr float SPEED_DIVISOR_FOR_STEP_SMOOTHNESS = 1; //18;

constexpr static float OPTIMIZE_FOR_LANE_NUMBER = 3;
constexpr static float MAX_NUM_LANES_SIMPLE = 5;

static StraightRoadSection TEST_LANES{ 
   (int)MAX_NUM_LANES_SIMPLE,
   400,
   { {-55, 0, 4}, {-30, 0, 7}, {10, 0, 7}, {40, 1, 8}, {70, 1, 8}, {90, 2, 8}, {150, 2, 7}, {210, 2, 6}, {250, 2, 5}, {300, 2, 4} } 
}; // TODO: Delete eventually.

class Env2D : public Failable {
public:
   inline Env2D(const size_t num_cars) : Env2D(num_cars, "Environment2DSimple") {}

   inline Env2D(const size_t num_cars, const std::string& name) : num_cars_(num_cars), Failable(name) 
   {
      agents_pos_x_.resize(num_cars_);
      agents_pos_y_.resize(num_cars_);
      agents_vx_rel_.resize(num_cars_);
      agents_vy_.resize(num_cars_);
      agents_ax_.resize(num_cars_);
   }

   inline static int getImageWidth(const int num_lanes)
   {
      return (RIGHT_MARGIN - LEFT_MARGIN) * STREET_LENGTH_FOR_IMG_DIM / OPTIMIZE_FOR_LANE_NUMBER;
   }

   inline static int getImageHeight()
   {
      return STREET_LENGTH_FOR_IMG_DIM * RELATION_FOR_IMG_DIM * (MAX_NUM_LANES_SIMPLE / OPTIMIZE_FOR_LANE_NUMBER) /*+ 40 * (OPTIMIZE_FOR_LANE_NUMBER - NUM_LANES_SIMPLE)*/;
   }

   inline int egoLane() const
   {
      return (int)std::round(ego_pos_y_);
   }

   inline void createOthersVecs(
      CarParsVec& others_vec, 
      std::map<int, std::pair<float, float>>& others_future_vec,
      const std::set<int>& agents_to_draw_arrows_for,
      StraightRoadSection& lane_structure,
      const DataPackPtr future_data
      ) const
   {
      const float LANE_CONSTANT{ ((float)lane_structure.getNumLanes() - 1) * 2 };

      for (int i{}; i < num_cars_; i++) {
         others_vec.push_back({ agents_pos_y_[i], agents_pos_x_[i], (int)((agents_vx_rel_[i] + ego_vx_) * SPEED_DIVISOR_FOR_STEP_SMOOTHNESS), i });

         if (future_data && agents_to_draw_arrows_for.count(i)) {
            others_future_vec.insert(
               {
                  i, { future_data->getSingleVal("veh___6" + std::to_string(i) + "9___.rel_pos"),
                  (LANE_CONSTANT - future_data->getSingleVal("veh___6" + std::to_string(i) + "9___.on_lane")) / 2 }
               }
            );
         }
      }

      if (future_data) {
         others_future_vec.insert(
            {
               -1, { 0,
               (LANE_CONSTANT - future_data->getSingleVal("ego.on_lane")) / 2}
            });
      }
   }

   mutable std::map<int, std::pair<float, float>> others_past_vec_{};
   mutable CarPars past_ego_{ CarPars{ -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<int>::min(), HighwayImage::EGO_MOCK_ID } };
   mutable int count_{ 0 };

   inline std::shared_ptr<Image> getBirdseyeView(
      const std::set<int>& agents_to_draw_arrows_for,
      const ExtraVehicleArgs& additional_var_vals,
      const DataPackPtr future_data,
      const bool start_pdf,
      const float crop_left,
      const float crop_right,
      const std::shared_ptr<RoadGraph>& road_graph) const
   {
      if (!outside_view_) {
         outside_view_ = std::make_shared<HighwayImage>(getImageWidth(MAX_NUM_LANES_SIMPLE), getImageHeight(), std::make_shared<Plain2DTranslator>(), road_graph->getMyRoad().getNumLanes());
      }

      if (start_pdf) {
         outside_view_->restartPDF();
         outside_view_->setCropLeftRightPDF(crop_left, crop_right);
      }

      outside_view_->fillImg(BROWN);

      CarParsVec others_current_vec{};
      std::map<int, std::pair<float, float>> others_future_vec{};

      //createOthersVecs(others_current_vec, others_future_vec, agents_to_draw_arrows_for, road_graph.getMyRoad(), future_data); // TODO: For whole road graph, not only single lane.

      //env.agents_pos_x_[vehicle_index] = traj_pos.second.at(PossibleParameter::pos_x) - ego_x;
      //env.agents_pos_y_[vehicle_index] = 2 + traj_pos.second.at(PossibleParameter::pos_y) / mc::trajectory_generator::LANE_WIDTH;
      
      outside_view_->paintRoadGraph(
         road_graph,
         { 500, 60 },
         additional_var_vals,
         true, 60, (float) road_graph->getMyRoad().getNumLanes() / 2.0f);

      return outside_view_;
   }

   inline std::shared_ptr<HighwayImage> getCockpitView(
      const std::set<int>& agents_to_draw_arrows_for,
      const ExtraVehicleArgs& additional_var_vals,
      const DataPackPtr future_data,
      const bool start_pdf,
      const std::shared_ptr<RoadGraph>& road_graph,
      const int width,
      const int height) const
   {
      const float mirror_width = width * mirror_size_percent_;
      const float mirror_height = height * mirror_size_percent_;
      const float mirror_left = width * mirror_pos_left_percent_of_screen_;
      const float mirror_top = height * mirror_pos_top_percent_of_screen_;
      constexpr static float mirror_frame_thickness = 10;
      auto trans_cpv{ std::make_shared<Plain3DTranslator>(false) };
      auto trans_cpvm{ std::make_shared<Plain3DTranslator>(true) };

      if (!cockpit_view_ || cockpit_view_->getWidth() != width || cockpit_view_->getHeight() != height) {
         cockpit_view_ = std::make_shared<HighwayImage>(width, height, trans_cpv, road_graph->getMyRoad().getNumLanes());
         cockpit_view_mirror_ = std::make_shared<HighwayImage>(mirror_width, mirror_height, trans_cpvm, road_graph->getMyRoad().getNumLanes());
      }

      if (start_pdf) {
         cockpit_view_->restartPDF();
         cockpit_view_mirror_->restartPDF();
      }

      CarParsVec others_vec{};
      std::map<int, std::pair<float, float>> others_future_vec{};
      //createOthersVecs(others_vec, others_future_vec, agents_to_draw_arrows_for, lane_structure, future_data);

      auto no_trans{std::make_shared<DefaultHighwayTranslator>()};
      cockpit_view_->setTranslator(no_trans);
      cockpit_view_->paintEarthAndSky({ (float)width, (float)height });
      cockpit_view_->setTranslator(trans_cpv);

      cockpit_view_mirror_->setTranslator(no_trans);
      cockpit_view_mirror_->paintEarthAndSky({ (float)mirror_width, (float)mirror_height });
      cockpit_view_mirror_->setTranslator(trans_cpvm);

      cockpit_view_->paintRoadGraph(
         road_graph,
         { 500, 120 },
         additional_var_vals,
         true);

      cockpit_view_mirror_->paintRoadGraph(
         road_graph,
         { 500, 120 },
         additional_var_vals,
         true);

      cockpit_view_->setTranslator(std::make_shared<DefaultHighwayTranslator>());
      cockpit_view_mirror_->setTranslator(std::make_shared<DefaultHighwayTranslator>());

      // Paint mirror
      cockpit_view_->fillRectangle(
         mirror_left - mirror_frame_thickness,
         mirror_top - mirror_frame_thickness,
         mirror_width + mirror_frame_thickness * 2,
         mirror_height + mirror_frame_thickness * 2,
         BLACK,
         false);

      cockpit_view_->fillRectangle(
         mirror_left - mirror_frame_thickness / 2,
         mirror_top - mirror_frame_thickness / 2,
         mirror_width + mirror_frame_thickness,
         mirror_height + mirror_frame_thickness,
         WHITE,
         false);

      cockpit_view_->insertImage(mirror_left, mirror_top, *cockpit_view_mirror_, false);
      // EO Paint mirror

      return cockpit_view_;
   }

   std::vector<float> agents_pos_x_{};
   std::vector<float> agents_pos_y_{};
   std::vector<float> agents_vx_rel_{};
   std::vector<float> agents_vy_{};
   std::vector<float> agents_ax_{};
   float ego_vx_{};
   mutable float ego_pos_x_{}; // TODO: Remove mutable.
   float ego_pos_y_{};
   float ego_ax_{};
   float ego_vy_{};
   size_t num_cars_{};

private:
   mutable std::shared_ptr<HighwayImage> outside_view_{ nullptr };
   mutable std::shared_ptr<HighwayImage> cockpit_view_{ nullptr };
   mutable std::shared_ptr<HighwayImage> cockpit_view_mirror_{ nullptr };   mutable float mirror_size_percent_{ 0.35f };
   mutable float mirror_pos_left_percent_of_screen_{ 0.64f };
   mutable float mirror_pos_top_percent_of_screen_{ 0.05f };
};
} // vfm
