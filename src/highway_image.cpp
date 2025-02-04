//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "simulation/highway_image.h"
#include "static_helper.h"

using namespace vfm;

static constexpr float LANE_MARKER_THICKNESS{ 0.08 };

vfm::HighwayImage::HighwayImage(const int width, const int height, const std::shared_ptr<HighwayTranslator> translator, const int num_lanes) 
   : Image(width, height), Failable("HighwayImage")
{
   setTranslator(translator);
   setupVPointFor3DPerspective(num_lanes);
}

vfm::HighwayImage::HighwayImage(const std::shared_ptr<HighwayTranslator> translator, const int num_lanes) : HighwayImage(0, 0, translator, num_lanes)
{}

// 3D-specific stuff (mainly for setting the v_point_). TODO: Get rid of such things that should go into the translator.
float getXOffsetCorrection(
   const float blx,
   const float brx,
   const float desired_v_point_x) {
   return std::max(blx, std::min(brx, desired_v_point_x));
}

float getLaneLeft(const float laneFromLeftFloat, const float lane_w_average_, const float blx, const float num_lanes) {
   // TODO: It would be nicer if this method were the most foundational means to determine the lane positions.
   // I.e., bl_.x should be calculated using this method, not the other way around. So far, it is
   // necessary to make sure that bl_ has been calculated (using "adjust_paint_stuff()") before calling this method.
   float x_val = blx;
   int laneFromLeft = static_cast<int>(std::min(laneFromLeftFloat, num_lanes - 1));
   float factor_last{ 1 };

   for (int i = 0; i < laneFromLeft; i++) {
      factor_last = 1/* getLaneWidthPercentage(i) / getScaleLoss() */;
      x_val += /* marker_w_ */ 0 + lane_w_average_ * factor_last;
   }

   float floating_rest = (laneFromLeftFloat - laneFromLeft) * lane_w_average_ * factor_last;

   return x_val + floating_rest;
}

float getLaneMid(const float laneFromLeft, const float lane_w_average_, const float blx, const float num_lanes) {
   float factor_last{ 1 };
   return getLaneLeft(laneFromLeft, lane_w_average_, blx, num_lanes) + /*marker_w_*/ 0 + lane_w_average_ * factor_last / 2.0;
}

void HighwayImage::setupVPointFor3DPerspective(const int num_lanes)
{
   // Outermost screen borders.
   Vec2D screen_tl_{ 0, 0 };
   Vec2D screen_br_{ (float) getWidth(), (float) getHeight() };
   Vec2D screen_tr_{ screen_br_.x, screen_tl_.y };
   Vec2D screen_bl_{ screen_tl_.x, screen_br_.y };

   // Street box borders: Calculate helper variables for the corners.
   float screen_width = screen_br_.x - screen_bl_.x;
   float screen_height = screen_bl_.y - screen_tl_.y;

   float street_w_max_{ 0 };
   float marker_w_{ 0 };

   for (size_t i = 0; i < num_lanes; i++) {
      street_w_max_ += /* getLaneWidthPercentage(i) */ 1 * /* MAX_LANE_WIDTH_ */ 5500 + marker_w_;
   }

   float pad_screen = std::max((screen_width - street_w_max_) / 2, 0.0f);

   float left = pad_screen;
   float right = screen_width - pad_screen;
   float final_pb_padding = std::max(30 /* PB_WIDTH_ */ - pad_screen, 0.0f);

   // Street box borders: Set the corners.
   Vec2D tl_{ left + final_pb_padding, screen_height / TOP_FACTOR };
   Vec2D br_{ right - final_pb_padding, screen_br_.y };
   Vec2D tr_{ br_.x, tl_.y };
   Vec2D bl_{ tl_.x, br_.y };

   float lane_w_average_{ (br_.x - tl_.x - marker_w_) / num_lanes - marker_w_ };

   tr_.x = (getLaneLeft(static_cast<int>(num_lanes), lane_w_average_, bl_.x, num_lanes) + marker_w_);
   br_.x = (getLaneLeft(static_cast<int>(num_lanes), lane_w_average_, bl_.x, num_lanes) + marker_w_);

   /*
    * Vanishing point's x value is the car center's x value if adjusting
    * perspective to ego (or else just the middle of the street box).
    */
   v_point_ = Vec2Df((/*ADJUST_PERSPECTIVE_TO_EGO_
               ? getLaneMid(signs_.ego_lane_index) + animation_offset_x_
               :*/ (tl_.x + tr_.x) / 2), screen_height / V_POINT_FACTOR);

   int ego_offset = 0;
   if (true /* ADJUST_PERSPECTIVE_TO_EGO_ */) {
      lane_w_average_ = (br_.x - tl_.x - marker_w_) / num_lanes - marker_w_;

      ego_offset = (getLaneMid(/* signs_.ego_lane_index */ 0, lane_w_average_, bl_.x, num_lanes) + /* animation_offset_x_ */ 0) - (br_.x - tl_.x) / 2.0 - final_pb_padding;
      tl_.x -= ego_offset;
      br_.x -= ego_offset;
      tr_.x -= ego_offset;
      bl_.x -= ego_offset;
   }

   v_point_.x = (getXOffsetCorrection(v_point_.x, bl_.x, br_.x));
}

void HighwayImage::paintEarthAndSky()
{
   Vec2D screen_tl_{ 0, 0 };
   Vec2D screen_br_{ (float)getWidth(), (float)getHeight() };
   Vec2D screen_tr_{ screen_br_.x, screen_tl_.y };
   Vec2D screen_bl_{ screen_tl_.x, screen_br_.y };
   float screen_height = screen_bl_.y - screen_tl_.y;
   float tl_y{ screen_height / TOP_FACTOR };
   float tr_y{ tl_y };

   fillQuad(
      screen_tl_,
      screen_tr_,
      screen_bl_,
      screen_br_,
      BROWN);

   // From top screen corners to top street border.
   fillQuad(
      screen_tl_,
      screen_tr_,
      { screen_tl_.x, tl_y },
      { screen_tr_.x, tr_y },
      Color(119, 209, 255, 255));

   // Paint border around all of it.
   rectangle(screen_tl_.x, screen_tl_.y, screen_br_.x - screen_tl_.x - 1, screen_br_.y - screen_tl_.y - 1, DARK_BLUE, false);
}
// EO 3D-specific stuff (mainly for setting the v_point_). TODO: Get rid of such things that should go into the translator.

void vfm::HighwayImage::setTranslator(const std::shared_ptr<VisTranslator> function)
{
   auto highway_translator{ std::dynamic_pointer_cast<HighwayTranslator>(function) };

   if (!highway_translator) {
      addError("VisTranslator needs to be HighwayTranslator for HighwayImage.");
   }

   Image::setTranslator(function);
   highway_translator_ = highway_translator;
}

std::shared_ptr<HighwayTranslator> vfm::HighwayImage::getHighwayTranslator() const
{
   return highway_translator_;
}

void vfm::Image::fillBlinker(const Vec2Df& pos, const double side)
{
   float cw = CAR_WIDTH / LANE_WIDTH;

   fillRectangle(
      pos.x - (CAR_LENGTH - INDICATOR_LENGTH) / 2,
      pos.y + side * (cw + INDICATOR_HEIGHT) / 2,
      INDICATOR_LENGTH,
      INDICATOR_HEIGHT,
      INDICATOR_ON_COLOR);

   rectangle(
      pos.x - (CAR_LENGTH - INDICATOR_LENGTH) / 2,
      pos.y + side * (cw + INDICATOR_HEIGHT) / 2,
      INDICATOR_LENGTH,
      INDICATOR_HEIGHT,
      BLACK);

   fillRectangle(
      pos.x + (CAR_LENGTH - INDICATOR_LENGTH) / 2,
      pos.y + side * (cw + INDICATOR_HEIGHT) / 2,
      INDICATOR_LENGTH,
      INDICATOR_HEIGHT,
      INDICATOR_ON_COLOR);

   rectangle(
      pos.x + (CAR_LENGTH - INDICATOR_LENGTH) / 2,
      pos.y + side * (cw + INDICATOR_HEIGHT) / 2,
      INDICATOR_LENGTH,
      INDICATOR_HEIGHT,
      BLACK);
}

void vfm::HighwayImage::paintHighwaySceneFromData(StraightRoadSection& lane_structure, const DataPack& data, const std::shared_ptr<DataPack> future_data)
{
   float ego_lane = 2 - data.getSingleVal("ego.on_lane") / 2;
   float ego_v = data.getSingleVal("ego.v");
   float veh0_lane = 2 - data.getSingleVal("veh___609___.on_lane") / 2;
   float veh1_lane = 2 - data.getSingleVal("veh___619___.on_lane") / 2;
   float veh0_rel_pos = data.getSingleVal("veh___609___.rel_pos") / 4;
   float veh1_rel_pos = data.getSingleVal("veh___619___.rel_pos") / 4;
   float future_veh0_lane = future_data ? 2 - future_data->getSingleVal("veh___609___.on_lane") / 2 : -1;
   float future_veh1_lane = future_data ? 2 - future_data->getSingleVal("veh___619___.on_lane") / 2 : -1;
   float future_veh0_rel_pos = future_data ? future_data->getSingleVal("veh___609___.rel_pos") / 4 : -1;
   float future_veh1_rel_pos = future_data ? future_data->getSingleVal("veh___619___.rel_pos") / 4 : -1;
   float veh0_v = data.getSingleVal("veh___609___.v");
   float veh1_v = data.getSingleVal("veh___619___.v");

   ExtraVehicleArgs var_vals;

   for (int i = 0; i < 10; i++) {
      std::string varname = "veh___6" + std::to_string(i) + "9___.turn_signals";
      if (data.isDeclared(varname)) {
         var_vals.insert({ varname, data.getSingleValAsEnumName(varname) });
      }
   }

   lane_structure.setNumLanes(3);

   paintHighwayScene(
      lane_structure,
      { ego_lane, 0, (int) ego_v },
      { { veh0_lane, veh0_rel_pos, (int) veh0_v },
        { veh1_lane, veh1_rel_pos, (int) veh1_v } },
      future_data ? std::map<int, std::pair<float, float>>
   { { 0, { future_veh0_rel_pos, future_veh0_lane } },
      { 1, { future_veh1_rel_pos, future_veh1_lane } } } : std::map<int, std::pair<float, float>>{},
      0, var_vals, true);
}

void createArrows(const float pos_x, const float future_pos_x, const float pos_y, const float future_pos_y, std::vector<Pol2Df>& arrow_polygons)
{
   static const float EPSILON = 0.3; // 0.5 is the distance of a straight lane change without relative movement in long direction.
   Pol2Df pol{};
   Pol2Df bez{};

   if (std::abs(future_pos_x - pos_x) > EPSILON || std::abs(future_pos_y - pos_y) > EPSILON) {
      float between1 = pos_x + (future_pos_x - pos_x) / 3;
      float between2 = pos_x + (future_pos_x - pos_x) * 2 / 3;
      bez.bezier({ pos_x, pos_y }, { between1, pos_y }, { between2, future_pos_y }, { future_pos_x, future_pos_y });

      const float WIDTH_REDUCTION = bez.getBoundingBox()->getWidth();
      const float MAX_REDUCTION = 30;
      const float MIN_REDUCTION = 10;
      const float THRESHOLD_REDUCTION = 1;
      const float THICKNESS_REDUCTION{ WIDTH_REDUCTION >= THRESHOLD_REDUCTION
         ? MAX_REDUCTION
         : MIN_REDUCTION + WIDTH_REDUCTION * (MAX_REDUCTION - MIN_REDUCTION) / THRESHOLD_REDUCTION
      };
      const float ARROW_HEAD_REDUCTION = 1 + (MAX_REDUCTION - THICKNESS_REDUCTION) / 2.5;

      auto thickness_func = [THICKNESS_REDUCTION](const Vec2Df& point_position, const int point_num) -> float {
         return (3 + ((float)point_num) * 7 / 100) / THICKNESS_REDUCTION;
      };

      pol.createArrow(bez, thickness_func, ARROW_END_PLAIN_LINE, ARROW_END_PPT_STYLE_1, { 1, 1 }, { 0.50, 1.0f / ARROW_HEAD_REDUCTION });
      arrow_polygons.push_back(pol);
      //arrow_polygons.push_back({ {pos_x - 2, pos_y - 2}, {future_pos_x - 2, future_pos_y - 2}, {future_pos_x + 2, future_pos_y + 2}, {pos_x + 2, pos_y + 2} });
   }
}

void findMinMax(const CarPars& agent, int& min_lane, int& max_lane)
{
   int ln = (int)agent.car_lane_;

   if (((float)ln) != agent.car_lane_) {
      ln += ln < 0 ? -1 : 1;
   }

   if (ln < min_lane) {
      min_lane = ln;
   }
   if (ln > max_lane) {
      max_lane = ln;
   }
}

static constexpr float LONG_FACTOR{ 5 };
static constexpr float CAR_FINAL_WIDTH{ CAR_WIDTH / LANE_WIDTH };

void HighwayImage::plotCar2D(const float thick, const Vec2Df& pos_car, const Color& fill_color, const Color& car_frame_color)
{
   const float thikko{ (-thick + 1) / 30 };

   if (thick > 1) {
      fillRectangle(pos_car.x, pos_car.y, CAR_LENGTH - thikko * LONG_FACTOR, CAR_FINAL_WIDTH - thikko, car_frame_color);
      fillRectangle(pos_car.x, pos_car.y, CAR_LENGTH, CAR_FINAL_WIDTH, fill_color);
   }
   else {
      fillRectangle(pos_car.x, pos_car.y, CAR_LENGTH, CAR_FINAL_WIDTH, CAR_COLOR);
      rectangle(pos_car.x, pos_car.y, CAR_LENGTH - thikko * LONG_FACTOR, CAR_FINAL_WIDTH - thikko, car_frame_color);
   }
}

void HighwayImage::plotCar3D(const Vec2Df& pos_car, const Color& fill_color, const Color& car_frame_color)
{
   auto pale_filling{ fill_color };
   auto less_pale_filling{ fill_color };
   pale_filling.a = 100;
   less_pale_filling.a = 150;

   const float ww{ CAR_LENGTH / 2 };
   const float hh{ CAR_FINAL_WIDTH / 2 };
   const float height{ CAR_HEIGHT };

   const Vec3D tlu{ pos_car.x - ww, pos_car.y - hh, height };
   const Vec3D bru{ pos_car.x + ww, pos_car.y + hh, height };
   const Vec3D tru{ bru.x, tlu.y, height };
   const Vec3D blu{ tlu.x, bru.y, height };
   const Vec3D tld{ pos_car.x - ww, pos_car.y - hh, 0 };
   const Vec3D brd{ pos_car.x + ww, pos_car.y + hh, 0 };
   const Vec3D trd{ brd.x, tld.y, 0 };
   const Vec3D bld{ tld.x, brd.y, 0 };

   const Pol3D top{ { tlu, tru, bru, blu } };
   const Pol3D back{ { tlu, tld, bld, blu } };
   const Pol3D front{ { bru, tru, trd, brd } };
   const Pol3D left{ { bru, blu, bld, brd } };
   const Pol3D right{ { tru, tlu, tld, trd } };

   fillPolygon(front, pale_filling);
   fillPolygon(left, pale_filling);
   fillPolygon(top, pale_filling);
   fillPolygon(right, pale_filling);
   fillPolygon(back, pale_filling);
   drawPolygon(front, BLACK);
   drawPolygon(left, BLACK);
   drawPolygon(right, BLACK);
   drawPolygon(top, BLACK);
   drawPolygon(back, BLACK);
}

void vfm::HighwayImage::paintHighwayScene(
   const CarPars& ego,
   const CarParsVec& others,
   const std::map<int, std::pair<float, float>>& future_positions_of_others,
   const float ego_offset_x,
   const std::map<std::string, std::string>& var_vals,
   const bool print_agent_ids)
{
   StraightRoadSection dummy{};
   paintHighwayScene(dummy, ego, others, future_positions_of_others, ego_offset_x, var_vals, print_agent_ids);
}

static constexpr float ARC_LENGTH = MIN_DISTANCE_BETWEEN_SEGMENTS / 2;
static constexpr float THICK{ 7.f };
static constexpr float THIN{ THICK / 1.6f };
static constexpr float BEZIER_LANEMARKER{ 0.1 };

void vfm::HighwayImage::doShoulder(
   const bool min_lane,
   const bool is_current_shoulder,
   const bool is_last_shoulder, // We actually draw the shoulder for the last, not current.
   const bool is_last_last_shoulder,
   const Vec2D& last_one,
   const Vec2D& new_one_raw,
   const bool has_extended,
   const float actual_arc_length)
{
   if (is_last_shoulder) {
      //drawPolygon({ { last_one.x, tl_orig.y }, { last_one.x, br_orig.y } }, BLUE);
      //drawPolygon({ { new_one.x, tl_orig.y }, { new_one.x, br_orig.y } }, BLUE);
      //drawPolygon({ { last_one.x, last_one.y }, { new_one.x, last_one.y + 1 } }, BLUE);

      const float one_constant{ min_lane ? 1.0f : -1.0f };

      Pol2D temp_arrow{};
      Pol2D temp_base_points{};
      Vec2D new_one{ new_one_raw };

      bool is_extending{ last_one.y == new_one.y && !is_current_shoulder };
      bool is_extending_early{ is_last_shoulder && (min_lane ? last_one.y > new_one.y : last_one.y < new_one.y) };

      if (is_last_last_shoulder && !is_extending) {
         temp_base_points.add({ last_one.x, last_one.y + one_constant });
      }
      else if (has_extended) {
         temp_base_points.add({ last_one.x - actual_arc_length, last_one.y + one_constant });
      }
      else {
         temp_base_points.bezier(
            last_one,
            { last_one.x + ARC_LENGTH / 2.0f, last_one.y },
            { last_one.x + ARC_LENGTH / 2.0f,  last_one.y + one_constant },
            { last_one.x + ARC_LENGTH, last_one.y + one_constant },
            BEZIER_LANEMARKER);

         temp_base_points.add({ last_one.x + ARC_LENGTH, last_one.y + one_constant });
      }

      if (is_extending_early) {
         new_one.x -= 2 * ARC_LENGTH;
      }

      temp_base_points.add({ new_one.x, last_one.y + one_constant });

      if (is_extending || is_extending_early) {
         temp_base_points.bezier(
            { new_one.x + 1, last_one.y + one_constant },
            { new_one.x + ARC_LENGTH / 2.0f, last_one.y + one_constant },
            { new_one.x + ARC_LENGTH / 2.0f,  last_one.y },
            { new_one.x + ARC_LENGTH, last_one.y },
            BEZIER_LANEMARKER);

         //temp_base_points.add({ last_one.x + ARC_LENGTH, last_one.y + one_constant });
      }

      temp_arrow.createArrow(getTranslator()->translatePolygon(temp_base_points), THIN);
      fillPolygon(plain_2d_translator_.reverseTranslate(temp_arrow), LANE_MARKER_COLOR);
   }
}

void vfm::HighwayImage::removeNonExistentLanesAndMarkShoulders(
   const StraightRoadSection& lane_structure,
   const CarPars& ego,
   const Vec2D& tl_orig,
   const Vec2D& br_orig)
{
   const float ego_abs_pos{ ego.car_rel_pos_ };

   std::map<int, std::vector<std::pair<float, float>>> shoulders;

   for (int i = 0; i < lane_structure.getNumLanes(); i++) {
      shoulders[i] = {};
   }

   for (const bool min_lane : {true, false}) { // We have to remove lanes from the left (min_lane) and right (!min_lane).
      const float ego_lane{ min_lane ? ego.car_lane_ + 0.5f : ego.car_lane_ - 0.5f };

      Pol2D overpaint{};
      Pol2D arrow{};
      const auto first_point{ lane_structure.getSegments().begin()->second };
      const auto last_point{ lane_structure.getSegments().rbegin()->second };

      const auto first_segment_x{ lane_structure.getSegments().empty() ? tl_orig.x : lane_structure.getSegments().begin()->second.getBegin() - ego_abs_pos };
      float temp_x1{ tl_orig.x };
      if (first_segment_x <= tl_orig.x) {
         temp_x1 = first_segment_x - 1;
      }

      Vec2D last_one{ temp_x1, (float)(min_lane ? first_point.getActualDrivableMinLane() : first_point.getActualDrivableMaxLane()) - ego_lane };
      Vec2D last_last_one{ last_one };
      bool is_last_shoulder{ false };
      bool is_last_last_shoulder{ false };
      float last_arc_length{ ARC_LENGTH };

      overpaint.add(last_one);

      for (const auto& segment : lane_structure.getSegments()) {
         const auto current_point{ segment.second };
         const float arc_length{ ARC_LENGTH };

         Vec2D new_one{ current_point.getBegin() - ego_abs_pos, (float)(min_lane ? current_point.getActualDrivableMinLane() : current_point.getActualDrivableMaxLane()) - ego_lane };
         const bool is_vanishing{ (min_lane ? last_one.y < new_one.y : last_one.y > new_one.y) }; // Currently unused, but the logic seems nice to have.
         last_one.x = current_point.getBegin() - ego_abs_pos - arc_length;

         //if (is_vanishing) {
            //new_one.x += 2 * ARC_LENGTH;
            //last_one.x += 2 * ARC_LENGTH;
         //}

         const bool is_current_shoulder{ min_lane ? segment.second.isMinLaneShoulder() : segment.second.isMaxLaneShoulder() };
         const bool has_extended{ min_lane ? last_last_one.y > last_one.y : last_last_one.y < last_one.y };
         bool is_extending{ last_one.y == new_one.y && !is_current_shoulder };

         doShoulder(min_lane, is_current_shoulder, is_last_shoulder, is_last_last_shoulder, last_one, new_one, has_extended, last_arc_length);

         if (is_current_shoulder) {
            shoulders[new_one.y].push_back({ segment.second.getBegin() - ego_abs_pos + ARC_LENGTH, -1 });
         }
         else if (is_last_shoulder) {
            shoulders[last_one.y].back().second = segment.second.getBegin() - ego_abs_pos - ARC_LENGTH;
         }

         if (new_one.y != last_one.y) {
            overpaint.bezier(last_one, { last_one.x + arc_length / 2, last_one.y }, { new_one.x - arc_length / 2, new_one.y }, new_one, BEZIER_LANEMARKER);
         }
         overpaint.add(new_one);
         is_last_last_shoulder = is_last_shoulder;
         is_last_shoulder = min_lane ? segment.second.isMinLaneShoulder() : segment.second.isMaxLaneShoulder();
         last_last_one = last_one;
         last_one = new_one;
         last_arc_length = arc_length;
      }

      doShoulder(min_lane, false, is_last_shoulder, is_last_last_shoulder, last_one, { br_orig.x, last_one.y }, false, last_arc_length);

      float temp_x2{ br_orig.x };
      if (!overpaint.points_.empty() && overpaint.points_.back().x >= br_orig.x) {
         temp_x2 = overpaint.points_.back().x + 1;
      }

      overpaint.add({ temp_x2, (float)(min_lane ? last_point.getActualDrivableMinLane() : last_point.getActualDrivableMaxLane()) - ego_lane });

      arrow.createArrow(plain_2d_translator_.translatePolygon(overpaint), THICK / 2000 * getWidth());

      // TODO: Thicker in z direction.
      //for (int i = 0; i < arrow.points_.size(); i++) {
      //   const bool first_round{ i < arrow.points_.size() / 2 };
      //   auto& point = arrow.points_.at(i);
      //   //const auto& point_before = arrow.points_.at(i - 1);
      //   //const bool going_down{ point.y - point_before.y >= 0 };
      //   float pos_temp{ std::abs(point.y) + 0.5f + (min_lane ? 1 : -1) * (first_round ? -THICK / 2 : THICK / 2) };
      //   float offset{ 2.0f * (bool)(std::min)(std::ceil(pos_temp) - pos_temp, pos_temp - std::floor(pos_temp)) };
      //   point.x += (min_lane ? 1 : -1) * (first_round ? offset : -offset);
      //}

      overpaint.add(0, min_lane ? tl_orig : Vec2D{ tl_orig.x, br_orig.y });
      overpaint.add(0, min_lane ? Vec2D{ br_orig.x, tl_orig.y } : br_orig);

      fillPolygon(overpaint, GRASS_COLOR);
      fillPolygon(plain_2d_translator_.reverseTranslate(arrow), LANE_MARKER_COLOR);

      //drawPolygon(overpaint, RED, true, true, true);
      //drawPolygon(arrow, BLUE, true, true, true);
   }

   // TODO: Shoulder markings.
   //for (const auto& lane_shoulders : shoulders) {
   //   float lane = lane_shoulders.first;
   //   constexpr static float sheer{ 6 };
   //   constexpr static float thick{ 1.5 };
   //   constexpr static float distance{ 5 };
   //   const float lane_up{ lane - 0.5f };
   //   const float lane_down{ lane + 0.5f };

   //   for (const auto& shoulder : lane_shoulders.second) {
   //      //fillRectangle({ shoulder.first,  }, { shoulder.second, lane + 0.5f }, WHITE, false);

   //      for (float i = shoulder.first; i < shoulder.second; i += distance) {
   //         fillPolygon({ {i, lane_up }, {i + thick, lane_up}, {i + thick + sheer, lane_down}, {i + sheer, lane_down} }, WHITE);
   //      }
   //   }
   //}
}

void vfm::HighwayImage::setPerspective(
   const float street_height, 
   const float num_lanes, 
   const float ego_offset_x, 
   const int min_lane, 
   const int max_lane,
   const float street_top,
   const float ego_car_lane)
{
   const float lw = street_height / num_lanes;
   const float factor = lw / LANE_WIDTH;

   highway_translator_->setHighwayData(factor, getWidth(), getHeight(), ego_offset_x, street_top, min_lane, max_lane, lw, ego_car_lane, v_point_);
   plain_2d_translator_.setHighwayData(factor, getWidth(), getHeight(), ego_offset_x, street_top, min_lane, max_lane, lw, ego_car_lane, v_point_);

   static constexpr float PI{ 3.14159265359 };

   float cnt{ std::max(cnt_, 0.0f) };
   highway_translator_->getPerspective()->setCameraX(5.4 + 0 * cnt / 20);
   highway_translator_->getPerspective()->setCameraY(0 * cnt / 20);
   highway_translator_->getPerspective()->setCameraZ(-175 - 0 * cnt * 15);
   highway_translator_->getPerspective()->setCameraRotationX(0);
   highway_translator_->getPerspective()->setCameraRotationY(6.19 + (highway_translator_->isMirrored() ? PI : 0));
   highway_translator_->getPerspective()->setCameraRotationZ(PI / 2.0);
   highway_translator_->getPerspective()->setDisplayWindowX(-2);
   highway_translator_->getPerspective()->setDisplayWindowY(0);
   highway_translator_->getPerspective()->setDisplayWindowZ(27);
   //if (cnt_ >= 0 && cnt_ < 80) {
   //   cnt_ += step_;
   //   step_ += 0.0003;
   //}
   //else if (cnt_ >= 80) {
   //   step_ -= 0.0003;
   //}
   //else {
   //   cnt_ += 0.4;
   //}
}

void vfm::HighwayImage::paintHighwayScene(
   StraightRoadSection& lane_structure,
   const CarPars& ego,
   const CarParsVec& others,
   const std::map<int, std::pair<float, float>>& future_positions_of_others,
   const float ego_offset_x,
   const ExtraVehicleArgs& var_vals,
   const bool print_agent_ids)
{
   const int gap_0_front = var_vals.count("ego.gaps___609___.i_agent_front") ? std::stoi(var_vals.at("ego.gaps___609___.i_agent_front")) : -1;
   const int gap_1_front = var_vals.count("ego.gaps___619___.i_agent_front") ? std::stoi(var_vals.at("ego.gaps___619___.i_agent_front")) : -1;
   const int gap_2_front = var_vals.count("ego.gaps___629___.i_agent_front") ? std::stoi(var_vals.at("ego.gaps___629___.i_agent_front")) : -1;
   const int gap_0_rear = var_vals.count("ego.gaps___609___.i_agent_rear") ? std::stoi(var_vals.at("ego.gaps___609___.i_agent_rear")) : -1;
   const int gap_1_rear = var_vals.count("ego.gaps___619___.i_agent_rear") ? std::stoi(var_vals.at("ego.gaps___619___.i_agent_rear")) : -1;
   const int gap_2_rear = var_vals.count("ego.gaps___629___.i_agent_rear") ? std::stoi(var_vals.at("ego.gaps___629___.i_agent_rear")) : -1;

   const float street_height = getHeight() * 0.8;
   const float street_top = (getHeight() - street_height) / 2;
   float num_lanes;

   int min_lane = lane_structure.isValid() ? 0 : -1;
   int max_lane = lane_structure.isValid() ? lane_structure.getNumLanes() - 1 : -1;

   if (min_lane < 0) {
      for (const auto& pair : others) {
         findMinMax(pair, min_lane, max_lane);
      }

      findMinMax(ego, min_lane, max_lane);
   }

   if (min_lane < 0) min_lane = 0;
   if (max_lane < 0) max_lane = 0;

   if (min_lane != 0) lane_structure.addError("Qua convention, min_lane shall be 0 at all times (it is currently " + std::to_string(min_lane) + ").");

   num_lanes = 1 + max_lane - min_lane;

   lane_structure.setNumLanes(num_lanes);
   lane_structure.cleanUp();

   setPerspective(street_height, num_lanes, ego_offset_x, min_lane, max_lane, street_top, ego.car_lane_);
   const Vec2Df pos_ego{ 0, 0 };  // Convention: Ego is at 0/0 position.
   float y = street_top;

   auto tl_orig{ plain_2d_translator_.reverseTranslate({ 0, 0 }).projectToXY() };
   auto br_orig{ plain_2d_translator_.reverseTranslate({ (float)getWidth() - 1, (float)getHeight() - 1}).projectToXY() };
   const float street_left_border{ min_lane - 0.5f - ego.car_lane_ };
   const float street_width{ (float)((max_lane - min_lane) + 1) };
   const float street_right_border{ street_left_border + street_width };

   constexpr static float METERS_TO_LOOK_AHEAD{ 130 };

   tl_orig.x = -METERS_TO_LOOK_AHEAD;
   br_orig.x = METERS_TO_LOOK_AHEAD;

   fillRectangle(tl_orig.x, street_left_border, br_orig.x - tl_orig.x, street_width, PAVEMENT_COLOR, false);

   for (int i = min_lane + 1; i <= max_lane; i++) {
      y = street_left_border + i - min_lane;
      dashed_line(tl_orig.x + offset_dashed_lines_on_highway_, y, br_orig.x, y, LANE_MARKER_THICKNESS, LANE_MARKER_COLOR, DASH_WIDTH);
   }

   offset_dashed_lines_on_highway_ = -ego.car_rel_pos_;

   while (offset_dashed_lines_on_highway_ >= br_orig.x) {
      offset_dashed_lines_on_highway_ -= br_orig.x;
   }

   removeNonExistentLanesAndMarkShoulders(lane_structure, ego, tl_orig, br_orig);

   // Plot other vehicles
   const Color CAR_FRAME_COLOR{ DARK_GREY };
   std::vector<Pol2Df> arrow_polygons{};

   std::map<int, float> cars_by_distance{};

   for (int i = 0; i < others.size(); i++) {
      Vec2Df pos{ others[i].car_rel_pos_, others[i].car_lane_ - ego.car_lane_ };
      cars_by_distance[i] = Vec2Df({ pos.x, std::abs(ego.car_lane_ - pos.y) * LANE_WIDTH }).length();
   }

   std::vector<int> cars_sorted_by_distance{};

   while (cars_sorted_by_distance.size() < cars_by_distance.size()) { // TODO: This is maybe the most inefficient way of doing the sorting there is...
      float max = -1;
      int max_id = -1;

      for (auto& el : cars_by_distance) {
         if (el.second > max) {
            max_id = el.first;
            max = el.second;
         }
      }

      if (max_id >= 0) {
         cars_by_distance.at(max_id) = -1;
         cars_sorted_by_distance.push_back(max_id);
      }
   }

   for (int i = 0; i < cars_sorted_by_distance.size(); i++) {
      auto car_id{ cars_sorted_by_distance[i] };
      const auto pair{ others[car_id] };

      Vec2Df pos{ pair.car_rel_pos_, pair.car_lane_ - ego.car_lane_ };

      float text_pos_x{ (std::max)(tl_orig.x + 2, (std::min)((float)br_orig.x - 2, pos.x)) };
      Color car_frame_color{ CAR_FRAME_COLOR };
      float thick{ 3 };

      //static const int SLIDE_X{ CAR_LENGTH * 1.2 };
      //static const int SLIDE_Y{ 0.5 };

      if (car_id == gap_0_front) { car_frame_color = GAP_0_FRONT_COLOR; thick = 5; /* writeAsciiText(text_pos_x - SLIDE_X, pos.y - SLIDE_Y, "0F"); */ }
      if (car_id == gap_1_front) { car_frame_color = GAP_1_FRONT_COLOR; thick = 5; /* writeAsciiText(text_pos_x - SLIDE_X, pos.y, "1F");           */ }
      if (car_id == gap_2_front) { car_frame_color = GAP_2_FRONT_COLOR; thick = 5; /* writeAsciiText(text_pos_x - SLIDE_X, pos.y + SLIDE_Y, "2F"); */ }
      if (car_id == gap_0_rear) { car_frame_color = GAP_0_REAR_COLOR; thick = 5;   /* writeAsciiText(text_pos_x + SLIDE_X, pos.y - SLIDE_Y, "0R"); */ }
      if (car_id == gap_1_rear) { car_frame_color = GAP_1_REAR_COLOR; thick = 5;   /* writeAsciiText(text_pos_x + SLIDE_X, pos.y, "1R");           */ }
      if (car_id == gap_2_rear) { car_frame_color = GAP_2_REAR_COLOR; thick = 5;   /* writeAsciiText(text_pos_x + SLIDE_X, pos.y + SLIDE_Y, "2R"); */ }

      if (future_positions_of_others.count(car_id)) {
         float future_pos_x{ pos_ego.x + future_positions_of_others.at(car_id).first };
         float future_pos_y{ pos_ego.y + (future_positions_of_others.at(car_id).second - ego.car_lane_) };
         createArrows(pos.x, future_pos_x, pos.y, future_pos_y, arrow_polygons);
      }

      std::string varname_turn_signals{ "veh___6" + std::to_string(car_id) + "9___.turn_signals" };
      if (var_vals.count(varname_turn_signals)) {
         if (var_vals.at(varname_turn_signals) == "LEFT") {
            fillBlinker(Vec2Df{ pos.x, pos.y }, -1.0);
         }
         else if (var_vals.at(varname_turn_signals) == "RIGHT") {
            fillBlinker(Vec2Df{ pos.x, pos.y }, 1.0);
         }
      }

      // vehicle speed number
      plotCar2D(thick, pos, CAR_COLOR, car_frame_color);
      writeAsciiText(text_pos_x, pos.y, std::to_string(pair.car_velocity_), CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLACK);

      if (print_agent_ids) {
         writeAsciiText(text_pos_x, pos.y - 0.5, std::to_string(car_id), CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLUE);
      }
   }

   std::string varname_ego_turn_signals = "ego.turn_signals";
   if (var_vals.count(varname_ego_turn_signals)) {
      if (var_vals.at(varname_ego_turn_signals) == "LEFT") {
         fillBlinker(Vec2Df{ pos_ego.x, pos_ego.y }, -1.0);
      }
      else if (var_vals.at(varname_ego_turn_signals) == "RIGHT") {
         fillBlinker(Vec2Df{ pos_ego.x, pos_ego.y }, 1.0);
      }
   }

   //// Ego speed number
   if (future_positions_of_others.count(-1)) {
      float future_pos_x = pos_ego.x + future_positions_of_others.at(-1).first;
      float future_pos_y = pos_ego.y + (future_positions_of_others.at(-1).second - ego.car_lane_);
      createArrows(pos_ego.x, future_pos_x, pos_ego.y, future_pos_y, arrow_polygons);
   }

   plotCar2D(3, pos_ego, RED, CAR_FRAME_COLOR);
   for (const auto& pol : arrow_polygons) {
      fillPolygon(pol, DARK_GREY);
   }

   for (int i = 0; i < cars_sorted_by_distance.size(); i++) {
      const auto pair = others[cars_sorted_by_distance[i]];
      Vec2Df pos{ pair.car_rel_pos_, pair.car_lane_ - ego.car_lane_ };
      if (getHighwayTranslator()->is3D()) plotCar3D(pos, CAR_COLOR, CAR_FRAME_COLOR);
   }

   if (getHighwayTranslator()->is3D()) plotCar3D(pos_ego, RED, CAR_FRAME_COLOR);

   writeAsciiText(pos_ego.x, pos_ego.y, std::to_string(ego.car_velocity_), CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLACK);

   auto text_pos_y{ plain_2d_translator_.reverseTranslate({ 0, 13 }).y };
   writeAsciiText(-CAR_LENGTH / 2, text_pos_y, std::to_string((int)ego.car_rel_pos_) + "m", CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLACK);

   float tl_orig_one_below_y{ plain_2d_translator_.reverseTranslate({0, 0}).y };
   rectangle(tl_orig.x, tl_orig_one_below_y, br_orig.x - tl_orig.x, br_orig.y - tl_orig.y, BLACK, false);
}
