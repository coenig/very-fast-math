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
   (int)MAX_NUM_LANES_SIMPLE,
   400,
   LANE_WIDTH_M,
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

   /// TODO: Future vec not yet working.
   /// TODO2: Road Graph does not need a modification here.
   inline void createOthersVecs2(
      std::map<int, std::pair<float, float>>& others_future_vec,
      const std::set<int>& agents_to_draw_arrows_for,
      const std::shared_ptr<RoadGraph> road_graph,
      const DataPackPtr future_data
   ) const
   {
      const float LANE_CONSTANT{ ((float)road_graph->getMyRoad().getNumActualLanes() - 1) * 2}; // TODO: What is different sections have different numbers of lanes?
      StraightRoadSection& ego_road{ road_graph->findSectionWithEgoIfAny()->getMyRoad() }; // TODO: Can be null??
      const CarDimensions dim{ ego_road.getEgo()->car_dim_ };

      ego_road.setEgo(std::make_shared<CarPars>(ego_pos_y_, ego_pos_x_, ego_vx_ * SPEED_DIVISOR_FOR_STEP_SMOOTHNESS, RoadGraph::EGO_MOCK_ID, dim));

      for (int i{}; i < num_cars_; i++) {
         CarParsVec others_vec{ road_graph->findSectionWithCar(i)->getMyRoad().getOthers() };
         others_vec.push_back({ agents_pos_y_[i], agents_pos_x_[i], (int)((agents_vx_rel_[i] + ego_vx_) * SPEED_DIVISOR_FOR_STEP_SMOOTHNESS), i, dim });
         road_graph->findSectionWithCar(i)->getMyRoad().setOthers(others_vec);

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
   mutable CarPars past_ego_{ CarPars{ -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), (std::numeric_limits<int>::min)(), RoadGraph::EGO_MOCK_ID, DEFAULT_CAR_DIMENSIONS_M } };
   mutable int count_{ 0 };

   inline std::shared_ptr<Image> getBirdseyeView(
      const std::set<int>& agents_to_draw_arrows_for,
      const ExtraVehicleArgs& additional_var_vals,
      const DataPackPtr future_data,
      const bool start_pdf,
      const float crop_left,
      const float crop_right,
      const std::shared_ptr<RoadGraph>& road_graph,
      const HighwayImage::PlainRoadMode paint_cars,
      const HighwayImage::CameraMode camera_mode = HighwayImage::CameraMode::ego_following) const
   {
      const bool infinite_highway{ false /*road_graph->getNodeCount() == 1*/ };

      // Copilot
      int canvas_width{ 5000 };
      int canvas_height{ getImageHeight() * (!infinite_highway ? 7 : 1) };

      if (camera_mode == HighwayImage::CameraMode::fit_to_roads) {
         // Fit the whole road graph to the standard birdseye width, with height following the
         // content's aspect ratio. This keeps the 2D view as large as the ego view (and the
         // stacked cockpit) while staying high-resolution, i.e. without the fixed-frame margins.
         constexpr float TARGET_WIDTH{ 5000.0f }; // Standard birdseye width (matches cockpit in combined view).
         constexpr float MAX_PPM{ 40.0f };        // Avoid absurd zoom on tiny single-section graphs.
         constexpr float PADDING_FACTOR{ 1.10f };
         constexpr int MAX_FIT_DIM{ 12000 };      // Safety cap against gigantic canvases.

         const Rec2D bb{ road_graph->getBoundingBox(false) }; // Exclude ghosts so the fit stays stable across frames.
         const float lane_width{ road_graph->getMyRoad().getLaneWidth() };
         float max_lanes{ 1.0f };
         for (const auto& node : road_graph->getAllNodes()) {
            max_lanes = (std::max)(max_lanes, static_cast<float>(node->getMyRoad().getNumActualLanes()));
         }
         const float lateral_margin{ max_lanes * lane_width / 2.0f + lane_width };
         const float content_w{ ((std::max)(1.0f, bb.getWidth()) + 2.0f * lateral_margin) * PADDING_FACTOR };
         const float content_h{ ((std::max)(1.0f, bb.getHeight()) + 2.0f * lateral_margin) * PADDING_FACTOR };

         float ppm{ (std::min)({ TARGET_WIDTH / content_w, MAX_PPM, MAX_FIT_DIM / content_h }) };
         canvas_width = (std::max)(1, static_cast<int>(content_w * ppm));
         canvas_height = (std::max)(1, static_cast<int>(content_h * ppm));
      }
      // EO Copilot
      
      if (true || !outside_view_) { // TODO: Can we optimize that for performance?
         outside_view_ = std::make_shared<HighwayImage>(
            canvas_width,
            canvas_height,
            std::make_shared<Plain2DTranslator>(), 
            road_graph->getMyRoad().getNumActualLanes());
      }

      if (start_pdf) {
         outside_view_->restartPDF();
         outside_view_->setCropLeftRightPDF(crop_left, crop_right);
      }

      if (paint_cars == HighwayImage::PlainRoadMode::regular) outside_view_->fillImg(BROWN);

      std::map<int, std::pair<float, float>> others_future_vec{}; // TODO: Future vec not yet working.
      //createOthersVecs2(others_future_vec, agents_to_draw_arrows_for, road_graph, future_data);
      
      Rec2D bounding_box{ road_graph->getBoundingBox() };
      const float offset_x{ 0 };
      const float offset_y{ 20 };

      outside_view_->paintRoadGraph(
         road_graph,
         { 500, 30 },
         paint_cars,
         additional_var_vals,
         true, offset_x, offset_y, camera_mode);

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
      //auto trans_cpvm{ std::make_shared<Plain3DTranslator>(true) };

      if (!cockpit_view_ || cockpit_view_->getWidth() != width || cockpit_view_->getHeight() != height) {
         cockpit_view_ = std::make_shared<HighwayImage>(width, height, trans_cpv, road_graph->getMyRoad().getNumActualLanes());
         //cockpit_view_mirror_ = std::make_shared<HighwayImage>(mirror_width, mirror_height, trans_cpvm, road_graph->getMyRoad().getNumLanes());
      }

      if (start_pdf) {
         cockpit_view_->restartPDF();
         //cockpit_view_mirror_->restartPDF();
      }

      std::map<int, std::pair<float, float>> others_future_vec{}; // TODO: Future vec not yet working.
      //createOthersVecs2(others_future_vec, agents_to_draw_arrows_for, road_graph, future_data);

      auto no_trans{std::make_shared<DefaultHighwayTranslator>()};
      cockpit_view_->setTranslator(no_trans);
      cockpit_view_->paintEarthAndSky(true, { (float)width, (float)height });
      cockpit_view_->setTranslator(trans_cpv);

      //cockpit_view_mirror_->setTranslator(no_trans);
      //cockpit_view_mirror_->paintEarthAndSky({ (float)mirror_width, (float)mirror_height });

      // The 3D perspective reference { 500, 120 } is calibrated for the default cockpit size
      // (2400x480); scale it with the actual canvas so the scene keeps the same framing (i.e.
      // doesn't look zoomed out) when the combined view renders the cockpit larger.
      constexpr float REFERENCE_COCKPIT_WIDTH{ 2400.0f };
      constexpr float REFERENCE_COCKPIT_HEIGHT{ 480.0f };
      const float perspective_x{ 500.0f * width / REFERENCE_COCKPIT_WIDTH };
      const float perspective_y{ 120.0f * height / REFERENCE_COCKPIT_HEIGHT };

      cockpit_view_->paintRoadGraph(
         road_graph,
         { perspective_x, perspective_y },
         HighwayImage::PlainRoadMode::regular, // Paint cars.
         additional_var_vals,
         true);

      //cockpit_view_mirror_->paintRoadGraph(
      //   road_graph,
      //   { 500, 120 },
      //   additional_var_vals,
      //   true);

      cockpit_view_->setTranslator(std::make_shared<DefaultHighwayTranslator>());
      //cockpit_view_mirror_->setTranslator(std::make_shared<DefaultHighwayTranslator>());

      // Paint mirror
      //cockpit_view_->fillRectangle(
      //   mirror_left - mirror_frame_thickness,
      //   mirror_top - mirror_frame_thickness,
      //   mirror_width + mirror_frame_thickness * 2,
      //   mirror_height + mirror_frame_thickness * 2,
      //   BLACK,
      //   false);

      //cockpit_view_->fillRectangle(
      //   mirror_left - mirror_frame_thickness / 2,
      //   mirror_top - mirror_frame_thickness / 2,
      //   mirror_width + mirror_frame_thickness,
      //   mirror_height + mirror_frame_thickness,
      //   WHITE,
      //   false);

      //cockpit_view_->insertImage(mirror_left, mirror_top, *cockpit_view_mirror_, false);
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
   //mutable std::shared_ptr<HighwayImage> cockpit_view_mirror_{ nullptr };
   mutable float mirror_size_percent_{ 0.35f };
   mutable float mirror_pos_left_percent_of_screen_{ 0.64f };
   mutable float mirror_pos_top_percent_of_screen_{ 0.05f };
};
} // vfm
