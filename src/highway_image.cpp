//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "simulation/highway_image.h"
#include "static_helper.h"
#include "testing/interactive_testing.h"
#include "geometry/bezier_functions.h"

using namespace vfm;

static constexpr float LANE_MARKER_THICKNESS{ 0.08 };

vfm::HighwayImage::HighwayImage(const int width, const int height, const std::shared_ptr<HighwayTranslator> translator, const int num_lanes) 
   : Image(width, height), Failable("HighwayImage")
{
   setTranslator(translator);
   setupVPointFor3DPerspective(num_lanes, { (float) width, (float) height });
}

vfm::HighwayImage::HighwayImage(const std::shared_ptr<HighwayTranslator> translator, const int num_lanes) 
   : HighwayImage(0, 0, translator, num_lanes)
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

void HighwayImage::setupVPointFor3DPerspective(const int num_lanes, const Vec2D& dim)
{
   // Outermost screen borders.
   Vec2D screen_tl_{ 0, 0 };
   Vec2D screen_br_{ dim.x, dim.y };
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

void HighwayImage::paintEarthAndSky(const Vec2D& dim_raw)
{
   const float dimx{ (float) getWidth() };
   const float dimy{ dimx / 5 };
   Vec2D dim{ dim_raw.length() == 0 ? Vec2D{ dimx, dimy } : dim_raw };

   Vec2D screen_tl_{ 0, 0 };
   Vec2D screen_br_{ dim.x, dim.y };
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

      pol.createArrow(bez, thickness_func, {}, {}, ARROW_END_PLAIN_LINE, ARROW_END_PPT_STYLE_1, { 1, 1 }, { 0.50, 1.0f / ARROW_HEAD_REDUCTION });
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

void HighwayImage::plotCar2D(const float thick, const Vec2Df& pos_car, const Color& fill_color, const Color& car_frame_color, const Vec2D scale, const float angle_rad)
{
   const float thikko{ (-thick + 1) / 30 };

   if (thick > 1) {
      float width1  = (CAR_LENGTH - thikko * LONG_FACTOR) * scale.x;
      float height1 = (CAR_FINAL_WIDTH - thikko) * scale.y;
      float width2  = (CAR_LENGTH) * scale.x;
      float height2 = (CAR_FINAL_WIDTH) * scale.y;
      float ww1 = width1 / 2;
      float hh1 = height1 / 2;
      float ww2 = width2 / 2;
      float hh2 = height2 / 2;

      Vec2D tl1{ pos_car.x - ww1, pos_car.y - hh1 };
      Vec2D br1{ pos_car.x + width1 - ww1, pos_car.y + height1 - hh1 };
      Vec2D tl2{ pos_car.x - ww2, pos_car.y - hh2 };
      Vec2D br2{ pos_car.x + width2 - ww2, pos_car.y + height2 - hh2 };
      Pol2D p1{ tl1, { tl1.x, br1.y }, br1, { br1.x, tl1.y } };
      Pol2D p2{ tl2, { tl2.x, br2.y }, br2, { br2.x, tl2.y } };

      p1.rotate(angle_rad);
      p2.rotate(angle_rad);

      fillPolygon(p1, car_frame_color);
      fillPolygon(p2, fill_color);
   }
   else {
      fillRectangle(pos_car.x, pos_car.y, CAR_LENGTH * scale.x, CAR_FINAL_WIDTH * scale.y, CAR_COLOR);
      rectangle(pos_car.x, pos_car.y, (CAR_LENGTH - thikko * LONG_FACTOR) * scale.x, (CAR_FINAL_WIDTH - thikko) * scale.y, car_frame_color);
   }
}

void HighwayImage::plotCar3D(const Vec2Df& pos_car, const Color& fill_color, const Color& car_frame_color, const Vec2D scale, const float angle_rad)
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
      fillPolygon(plain_2d_translator_->reverseTranslate(temp_arrow), LANE_MARKER_COLOR);
   }
}

static constexpr int FIRST_LANE_CONNECTOR_ID{ 5 };

void vfm::HighwayImage::removeNonExistentLanesAndMarkShoulders(
   const StraightRoadSection& lane_structure,
   const std::shared_ptr<CarPars> ego_raw,
   const Vec2D& tl_orig,
   const Vec2D& br_orig,
   const bool infinite_road,
   const Vec2D& dim,
   const bool infinitesimal_road,
   std::vector<ConnectorPolygonEnding>& connections)
{
   auto ego = ego_raw ? *ego_raw : CarPars{};

   std::map<int, std::vector<std::pair<float, float>>> shoulders;

   for (int i = 0; i < lane_structure.getNumLanes(); i++) {
      shoulders[i] = {};
   }

   for (const bool min_lane : { true, false }) { // We have to remove lanes from the left (min_lane) and right (!min_lane).
      const float ego_lane{ min_lane ? ego.car_lane_ + 0.5f : ego.car_lane_ - 0.5f };

      Pol2D overpaint{};
      Pol2D arrow{};
      const auto first_point{ lane_structure.getSegments().begin()->second };
      const auto last_point{ lane_structure.getSegments().rbegin()->second };

      const auto first_segment_x{ lane_structure.getSegments().empty() ? tl_orig.x : lane_structure.getSegments().begin()->second.getBegin() };
      float temp_x1{ tl_orig.x };
      if (first_segment_x <= tl_orig.x) {
         temp_x1 = first_segment_x - 1;
      }

      Vec2D last_one{ temp_x1, (float)(min_lane ? first_point.getActualDrivableMinLane() : first_point.getActualDrivableMaxLane()) - ego_lane };
      Vec2D last_last_one{ last_one };
      bool is_last_shoulder{ false };
      bool is_last_last_shoulder{ false };
      float last_arc_length{ ARC_LENGTH };

      if (infinite_road) overpaint.add(last_one); // TODO: Not sure what this was about, looks good without when no infinite_road. Might go away overall...?

      for (const auto& segment : lane_structure.getSegments()) {
         const auto current_point{ segment.second };
         const float arc_length{ ARC_LENGTH };

         Vec2D new_one{ current_point.getBegin(), (float)(min_lane ? current_point.getActualDrivableMinLane() : current_point.getActualDrivableMaxLane()) - ego_lane };
         const bool is_vanishing{ (min_lane ? last_one.y < new_one.y : last_one.y > new_one.y) }; // Currently unused, but the logic seems nice to have.
         last_one.x = current_point.getBegin() - arc_length;

         //if (is_vanishing) {
            //new_one.x += 2 * ARC_LENGTH;
            //last_one.x += 2 * ARC_LENGTH;
         //}

         const bool is_current_shoulder{ min_lane ? segment.second.isMinLaneShoulder() : segment.second.isMaxLaneShoulder() };
         const bool has_extended{ min_lane ? last_last_one.y > last_one.y : last_last_one.y < last_one.y };
         bool is_extending{ last_one.y == new_one.y && !is_current_shoulder };

         doShoulder(min_lane, is_current_shoulder, is_last_shoulder, is_last_last_shoulder, last_one, new_one, has_extended, last_arc_length);

         if (is_current_shoulder) {
            shoulders[new_one.y].push_back({ segment.second.getBegin() + ARC_LENGTH, -1 });
         }
         else if (is_last_shoulder) {
            shoulders[last_one.y].back().second = segment.second.getBegin() - ARC_LENGTH;
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
      if (infinite_road && !overpaint.points_.empty() && overpaint.points_.back().x >= br_orig.x) {
         temp_x2 = overpaint.points_.back().x + 1;
      }

      overpaint.add({ temp_x2, (float)(min_lane ? last_point.getActualDrivableMinLane() : last_point.getActualDrivableMaxLane()) - ego_lane });

      // Drains and sources.
      // Assuming we do "min_lane = true" first.
      Vec2D fix{ (float)infinitesimal_road, 0 }; // For zero-lengh sections.

      if (min_lane) { // TOP
         auto top_right_corner = (*overpaint.points_.rbegin());
         auto top_left_corner = (*overpaint.points_.begin());
         auto top_right_second = (*(overpaint.points_.rbegin() + 1)) - fix;
         auto top_left_second = (*(overpaint.points_.begin() + 1)) + fix;

         //if (!getHighwayTranslator()->is3D()) {
            top_right_corner.add({ 0, ego.car_lane_ });
            top_right_second.add({ 0, ego.car_lane_ });
            top_left_corner.add({ 0, ego.car_lane_ });
            top_left_second.add({ 0, ego.car_lane_ });
         //}

         connections.insert(connections.begin(), ConnectorPolygonEnding{ // This is the frame border around the pavement
            ConnectorPolygonEnding::Side::drain,
            Lin2D{ top_right_corner, top_right_second }, // outgoing (temp; will be adjusted when bottom part becomes available)
            0, // temp thickness; will be calculated when bottom part becomes available
            std::make_shared<Color>(0, 0, 0, 0),  // Special encoding for frame border in LANE_MARKER_COLOR
            3,
            getHighwayTranslator()->is3D() ? plain_2d_translator_wrapped_ : getHighwayTranslator() });

         connections.insert(connections.begin(), ConnectorPolygonEnding{ // This is the frame border around the pavement
            ConnectorPolygonEnding::Side::source,
            Lin2D{ top_left_corner, top_left_second }, // incoming (temp; will be adjusted when bottom part becomes available)
            0, // temp thickness; will be calculated when bottom part becomes available
            std::make_shared<Color>(0, 0, 0, 0), // Special encoding for frame border in LANE_MARKER_COLOR
            3,
            getHighwayTranslator()->is3D() ? plain_2d_translator_wrapped_ : getHighwayTranslator() });

         connections.insert(connections.begin(), ConnectorPolygonEnding{ // Actual pavement
            ConnectorPolygonEnding::Side::drain,
            Lin2D{ top_right_corner, top_right_second }, // outgoing (temp; will be adjusted when bottom part becomes available)
            0, // temp thickness; will be calculated when bottom part becomes available
            std::make_shared<Color>(PAVEMENT_COLOR),
            4,
            getHighwayTranslator()->is3D() ? plain_2d_translator_wrapped_ : getHighwayTranslator() });

         connections.insert(connections.begin(), ConnectorPolygonEnding{ // Actual pavement
            ConnectorPolygonEnding::Side::source,
            Lin2D{ top_left_corner, top_left_second }, // incoming (temp; will be adjusted when bottom part becomes available)
            0, // temp thickness; will be calculated when bottom part becomes available
            std::make_shared<Color>(PAVEMENT_COLOR),
            4,
            getHighwayTranslator()->is3D() ? plain_2d_translator_wrapped_ : getHighwayTranslator() });

         for (int i = FIRST_LANE_CONNECTOR_ID; i < FIRST_LANE_CONNECTOR_ID + lane_structure.getNumLanes(); i++) { // Lane center lines
            const float lane_width{ getHighwayTranslator()->translate({0, LANE_WIDTH}).length() };
            auto top_right_corner_lane = top_right_corner;
            auto top_right_second_lane = top_right_second;
            auto top_left_corner_lane = top_left_corner;
            auto top_left_second_lane = top_left_second;
            const auto adjustment_for_missing_lanes_drain = lane_structure.getSegments().rbegin()->second.getMinLane() / 2;
            const auto adjustment_for_missing_lanes_source = lane_structure.getSegments().begin()->second.getMinLane() / 2;
            top_right_corner_lane.add({ 0, (float)(i - FIRST_LANE_CONNECTOR_ID + 0.5 - adjustment_for_missing_lanes_drain) });
            top_right_second_lane.add({ 0, (float)(i - FIRST_LANE_CONNECTOR_ID + 0.5 - adjustment_for_missing_lanes_drain) });
            top_left_corner_lane.add({ 0, (float)(i - FIRST_LANE_CONNECTOR_ID + 0.5 - adjustment_for_missing_lanes_source) });
            top_left_second_lane.add({ 0, (float)(i - FIRST_LANE_CONNECTOR_ID + 0.5 - adjustment_for_missing_lanes_source) });

            connections.insert(connections.begin(), ConnectorPolygonEnding{
               ConnectorPolygonEnding::Side::drain,
               Lin2D{ top_right_corner_lane, top_right_second_lane },
               0.1, // THICK
               std::make_shared<Color>(GREY),
               i,
               getHighwayTranslator()->is3D() ? plain_2d_translator_wrapped_ : getHighwayTranslator() });

            connections.insert(connections.begin(), ConnectorPolygonEnding{
               ConnectorPolygonEnding::Side::source,
               Lin2D{ top_left_corner_lane, top_left_second_lane },
               0.1, // THICK
               std::make_shared<Color>(GREY),
               i,
               getHighwayTranslator()->is3D() ? plain_2d_translator_wrapped_ : getHighwayTranslator() });
         }
      }
      else { // BOTTOM
         auto bottom_right_corner = (*overpaint.points_.rbegin());
         auto bottom_left_corner = (*overpaint.points_.begin());
         auto bottom_right_second = (*(overpaint.points_.rbegin() + 1)) - fix;
         auto bottom_left_second = (*(overpaint.points_.begin() + 1)) + fix;

         //if (!getHighwayTranslator()->is3D()) {
            bottom_right_corner.add({ 0, ego.car_lane_ });
            bottom_right_second.add({ 0, ego.car_lane_ });
            bottom_left_corner.add({ 0, ego.car_lane_ });
            bottom_left_second.add({ 0, ego.car_lane_ });
         //}

         for (auto& connection : connections) {
            if (connection.id_ == 3 || connection.id_ == 4) {
               const float MAGIC_NUMBER{ (0.034f + (lane_structure.getNumLanes() - 1) * 0.0125f) * 11 }; // TODO: For roundabout it was 0.0125...
               Vec2D middle_basepoint{ connection.connector_.base_point_ };
               Vec2D middle_second{ connection.connector_.direction_ };

               if (connection.side_ == ConnectorPolygonEnding::Side::drain) {
                  connection.thick_ = (float)lane_structure.getSegments().rbegin()->second.getNumLanes() * (LANE_WIDTH - LANE_MARKER_THICKNESS); // TODO: Something is not QUITE exact here
                  middle_basepoint.add(bottom_right_corner);
                  middle_second.add(bottom_right_second);
               }
               else if (connection.side_ == ConnectorPolygonEnding::Side::source) {
                  connection.thick_ = (float)lane_structure.getSegments().begin()->second.getNumLanes() * (LANE_WIDTH - LANE_MARKER_THICKNESS); // TODO: Something is not QUITE exact here
                  middle_basepoint.add(bottom_left_corner);
                  middle_second.add(bottom_left_second);
               }
               else {
                  addFatalError("Invalid connector type.");
               }

               middle_basepoint.div(2);
               middle_second.div(2);
               connection.connector_ = { middle_basepoint, middle_second };
            }
         }
      }
      // EO Drains and sources.

      const Vec2D tmp_root{ 0, 0 };
      const Vec2D tmp_away{ 0, LANE_MARKER_THICKNESS };
      const float LANE_MARKER_THICKNESS_TRANSLATED{ plain_2d_translator_->translate(tmp_root).distance(plain_2d_translator_->translate(tmp_away)) };
      arrow.createArrow(plain_2d_translator_->translatePolygon(overpaint), LANE_MARKER_THICKNESS_TRANSLATED);

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
      fillPolygon(plain_2d_translator_->reverseTranslate(arrow), LANE_MARKER_COLOR);

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
   const float ego_car_lane, 
   const Vec2D& dim)
{
   const float lw = street_height / num_lanes;
   const float factor = lw / LANE_WIDTH;

   highway_translator_->setHighwayData(factor, dim.x, dim.y, ego_offset_x, street_top, min_lane, max_lane, lw, ego_car_lane, v_point_);
   plain_2d_translator_->setHighwayData(factor, dim.x, dim.y, ego_offset_x, street_top, min_lane, max_lane, lw, ego_car_lane, v_point_);
   if (plain_2d_translator_wrapped_) plain_2d_translator_wrapped_->setHighwayData(factor, dim.x, dim.y, ego_offset_x, street_top, min_lane, max_lane, lw, ego_car_lane, v_point_);

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

void vfm::HighwayImage::paintStraightRoadSceneFromData(StraightRoadSection& lane_structure, const DataPack& data, const std::shared_ptr<DataPack> future_data)
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
   lane_structure.setEgo(std::make_shared<CarPars>(ego_lane, 0, (int)ego_v, RoadGraph::EGO_MOCK_ID));
   lane_structure.setOthers({ { veh0_lane, veh0_rel_pos, (int)veh0_v, 0 }, { veh1_lane, veh1_rel_pos, (int)veh1_v, 1 } });
   lane_structure.setFuturePositionsOfOthers(future_data ? std::map<int, std::pair<float, float>>
   { { 0, { future_veh0_rel_pos, future_veh0_lane } },
      { 1, { future_veh1_rel_pos, future_veh1_lane } } } : std::map<int, std::pair<float, float>>{});

   paintStraightRoadScene(lane_structure, true, 0, var_vals, true, { (float) getWidth(), (float)getHeight() });
}

void vfm::HighwayImage::paintStraightRoadSceneSimple(
   const CarPars& ego,
   const CarParsVec& others,
   const std::map<int, std::pair<float, float>>& future_positions_of_others,
   const float ego_offset_x,
   const std::map<std::string, std::string>& var_vals,
   const bool print_agent_ids)
{
   StraightRoadSection dummy{};

   dummy.setEgo(std::make_shared<CarPars>(ego.car_lane_, ego.car_rel_pos_, ego.car_velocity_, RoadGraph::EGO_MOCK_ID));
   dummy.setOthers(others);
   dummy.setFuturePositionsOfOthers(future_positions_of_others);

   paintStraightRoadScene(dummy, true, ego_offset_x, var_vals, print_agent_ids, { (float) getWidth(), (float) getHeight() });
}

std::vector<ConnectorPolygonEnding> vfm::HighwayImage::paintStraightRoadScene(
   StraightRoadSection& lane_structure,
   const bool infinite_road,
   const float ego_offset_x,
   const ExtraVehicleArgs& var_vals,
   const bool print_agent_ids, 
   const Vec2D& dim,
   const RoadDrawingMode mode)
{
   std::vector<ConnectorPolygonEnding> res{};

   const int gap_0_front = var_vals.count("ego.gaps___609___.i_agent_front") ? std::stoi(var_vals.at("ego.gaps___609___.i_agent_front")) : -1;
   const int gap_1_front = var_vals.count("ego.gaps___619___.i_agent_front") ? std::stoi(var_vals.at("ego.gaps___619___.i_agent_front")) : -1;
   const int gap_2_front = var_vals.count("ego.gaps___629___.i_agent_front") ? std::stoi(var_vals.at("ego.gaps___629___.i_agent_front")) : -1;
   const int gap_0_rear = var_vals.count("ego.gaps___609___.i_agent_rear") ? std::stoi(var_vals.at("ego.gaps___609___.i_agent_rear")) : -1;
   const int gap_1_rear = var_vals.count("ego.gaps___619___.i_agent_rear") ? std::stoi(var_vals.at("ego.gaps___619___.i_agent_rear")) : -1;
   const int gap_2_rear = var_vals.count("ego.gaps___629___.i_agent_rear") ? std::stoi(var_vals.at("ego.gaps___629___.i_agent_rear")) : -1;

   const float street_height = dim.y * 0.8;
   const float street_top = (dim.y - street_height) / 2;
   float num_lanes;

   auto ego = lane_structure.getEgo();
   auto ego_lane = ego ? ego->car_lane_ : 0;
   const auto ego_rel_pos = ego ? ego->car_rel_pos_ : 0;
   const auto ego_velocity = ego ? ego->car_velocity_ : 0;
   const auto others = lane_structure.getOthers();
   const auto future_positions_of_others = lane_structure.getFuturePositionsOfOthers();
   const auto road_length = infinite_road ? 300 : lane_structure.getLength();
   const float road_begin = infinite_road ? -300 : 0;
   const auto fix_for_connections = !road_length;

   int min_lane = lane_structure.isValid() ? 0 : -1;
   int max_lane = lane_structure.isValid() ? lane_structure.getNumLanes() - 1 : -1;

   if (min_lane < 0) {
      for (const auto& pair : others) {
         findMinMax(pair, min_lane, max_lane);
      }

      if (ego)
         findMinMax(*lane_structure.getEgo(), min_lane, max_lane);
   }

   if (min_lane < 0) min_lane = 0;
   if (max_lane < 0) max_lane = 0;

   if (min_lane != 0) lane_structure.addError("Qua convention, min_lane shall be 0 at all times (it is currently " + std::to_string(min_lane) + ").");

   num_lanes = 1 + max_lane - min_lane;

   lane_structure.setNumLanes(num_lanes);
   lane_structure.cleanUp(false);

   setPerspective(street_height, num_lanes, ego_offset_x, min_lane, max_lane, street_top, ego_lane, dim);
   float y = street_top;

   auto tl_orig{ plain_2d_translator_->reverseTranslate({ 0, 0 }).projectToXY() };
   auto br_orig{ plain_2d_translator_->reverseTranslate({ dim.x - 1, dim.y - 1}).projectToXY() };
   const float street_left_border{ min_lane - 0.5f - ego_lane };
   const float street_width{ (float)((max_lane - min_lane) + 1) };
   const float street_right_border{ street_left_border + street_width };

   constexpr static float METERS_TO_LOOK_BEHIND{ 130 };
   constexpr static float METERS_TO_LOOK_AHEAD{ METERS_TO_LOOK_BEHIND };

   tl_orig.x = -METERS_TO_LOOK_BEHIND;
   br_orig.x = METERS_TO_LOOK_AHEAD;

   if (mode == RoadDrawingMode::road || mode == RoadDrawingMode::both) {
      if (infinite_road) {
         fillRectangle(tl_orig.x, street_left_border, br_orig.x - tl_orig.x, street_width, PAVEMENT_COLOR, false);
      }
      else {
         fillRectangle(road_begin, street_left_border, road_length, street_width, PAVEMENT_COLOR, false);
      }
   }

   for (int i = min_lane + 1; i <= max_lane; i++) {
      y = street_left_border + i - min_lane;
      if (mode == RoadDrawingMode::road || mode == RoadDrawingMode::both) {
         dashed_line(road_begin, y, road_length, y, LANE_MARKER_THICKNESS, LANE_MARKER_COLOR, DASH_WIDTH);
      }
   }

   if (mode == RoadDrawingMode::road || mode == RoadDrawingMode::both) {
      removeNonExistentLanesAndMarkShoulders(
         lane_structure, ego, { road_begin, tl_orig.y }, { road_length, br_orig.y }, infinite_road, dim, fix_for_connections, res);
   }

   if (mode == RoadDrawingMode::cars || mode == RoadDrawingMode::both) {
      // Plot other vehicles
      const Color CAR_FRAME_COLOR{ DARK_GREY };
      std::vector<Pol2Df> arrow_polygons{};

      std::map<int, float> cars_by_distance{};
      std::map<int, int> id_to_others_vec{};

      for (int i = 0; i < others.size(); i++) {
         Vec2Df pos{ others[i].car_rel_pos_, others[i].car_lane_ - ego_lane };
         cars_by_distance[others[i].car_id_] = Vec2Df({ pos.x, std::abs(ego_lane - pos.y) * LANE_WIDTH }).length();
         id_to_others_vec[others[i].car_id_] = i;
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
         const auto pair{ others[id_to_others_vec[car_id]] };

         Vec2Df pos{ pair.car_rel_pos_, pair.car_lane_ - ego_lane };

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
            float future_pos_x{ future_positions_of_others.at(car_id).first };
            float future_pos_y{ (future_positions_of_others.at(car_id).second - ego_lane) };
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

         plotCar2D(thick, pos, car_id == 0 ? EGO_COLOR : CAR_COLOR, car_frame_color);

         // vehicle speed number
         writeAsciiText(text_pos_x, pos.y, std::to_string(pair.car_velocity_), CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLACK);

         if (print_agent_ids) {
            writeAsciiText(text_pos_x, pos.y - 0.5, std::to_string(car_id), CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLUE);
         }
      }

      std::string varname_ego_turn_signals = "ego.turn_signals";
      if (var_vals.count(varname_ego_turn_signals)) {
         if (var_vals.at(varname_ego_turn_signals) == "LEFT") {
            fillBlinker(Vec2Df{ 0, 0 }, -1.0);
         }
         else if (var_vals.at(varname_ego_turn_signals) == "RIGHT") {
            fillBlinker(Vec2Df{ 0, 0 }, 1.0);
         }
      }

      if (future_positions_of_others.count(-1)) {
         float future_pos_x = future_positions_of_others.at(-1).first;
         float future_pos_y = (future_positions_of_others.at(-1).second - ego_lane);
         createArrows(0, future_pos_x, 0, future_pos_y, arrow_polygons);
      }

      if (ego) plotCar2D(3, { ego_rel_pos, 0 }, RED, CAR_FRAME_COLOR);

      for (const auto& pol : arrow_polygons) {
         fillPolygon(pol, DARK_GREY);
      }

      for (int i = 0; i < cars_sorted_by_distance.size(); i++) {
         const auto pair = others[id_to_others_vec[cars_sorted_by_distance[i]]];
         Vec2Df pos{ pair.car_rel_pos_, pair.car_lane_ - ego_lane };
         if (getHighwayTranslator()->is3D()) plotCar3D(pos, CAR_COLOR, CAR_FRAME_COLOR);
      }

      if (ego) if (getHighwayTranslator()->is3D()) plotCar3D({ 0, 0 }, RED, CAR_FRAME_COLOR); // EGO 3D

      if (ego) writeAsciiText(ego_rel_pos, 0, std::to_string(ego_velocity), CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLACK);

      if (ego) {
         auto text_pos_y{ plain_2d_translator_->reverseTranslate({ ego_rel_pos, 13 }).y };
         writeAsciiText(-CAR_LENGTH / 2, text_pos_y, std::to_string((int)ego->car_rel_pos_) + "m", CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLACK);
      }
   }

   const auto reverse_origin_2D{ plain_2d_translator_->reverseTranslate({0, 0}) };
   float tl_orig_one_below_y{ reverse_origin_2D.y };
   //rectangle(road_begin - ego_rel_pos, tl_orig_one_below_y, road_length, br_orig.y - tl_orig.y, BLACK, false);

   // Drains and sources.
   const Vec2D fix{ (float) fix_for_connections, 0 };
   const auto bottom = tl_orig.y + br_orig.y - tl_orig.y + ego_lane;
   const auto top = tl_orig.y                            + ego_lane;
   const auto left = road_begin;
   const auto right = left + road_length;
   const auto bottom_right_corner = Vec2D{ right, bottom };
   const auto bottom_left_corner  = Vec2D{ left, bottom };
   const auto top_right_corner    = Vec2D{ right, top };
   const auto top_left_corner     = Vec2D{ left, top };
   auto middle_left = bottom_left_corner;
   middle_left.add(top_left_corner);
   middle_left.div(2);
   auto middle_right = bottom_right_corner;
   middle_right.add(top_right_corner);
   middle_right.div(2);

   res.insert(res.begin(), ConnectorPolygonEnding{
      ConnectorPolygonEnding::Side::drain,
      Lin2D{ middle_right, middle_left - fix }, // Outgoing
      bottom_left_corner.distance(top_left_corner) * 3.75f,
      std::make_shared<Color>(GRASS_COLOR),
      0,
      getHighwayTranslator()->is3D() ? plain_2d_translator_wrapped_ : getHighwayTranslator() });

   res.insert(res.begin(), ConnectorPolygonEnding{
      ConnectorPolygonEnding::Side::source,
      Lin2D{ middle_left, middle_right + fix }, // Incoming
      bottom_left_corner.distance(top_left_corner) * 3.75f,
      std::make_shared<Color>(GRASS_COLOR),
      0,
      getHighwayTranslator()->is3D() ? plain_2d_translator_wrapped_ : getHighwayTranslator() } );

   return res;
}

void vfm::HighwayImage::paintRoadGraph(
   const std::shared_ptr<RoadGraph> r_raw,
   const Vec2D& dim_raw_raw,
   const std::map<std::string, std::string>& var_vals,
   const bool print_agent_ids,
   const float TRANSLATE_X_raw,
   const float TRANSLATE_Y_raw)
{
   auto my_r = PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ ? vfm::test::paintExampleRoadGraphRoundabout(false, r_raw) : r_raw;

   my_r->normalizeRoadGraphToEgo();
   auto old_trans = getHighwayTranslator();
   const auto all_nodes = my_r->getAllNodes();

   const bool infinite_road{ all_nodes.size() == 1 && my_r->isRootedInZeroAndUnturned() }; // Only a single section, at root position and unturned, will be painted as infinite.
   float TRANSLATE_X{ TRANSLATE_X_raw };
   float TRANSLATE_Y{ TRANSLATE_Y_raw };
   Vec2D dim_raw{ dim_raw_raw };

   if (infinite_road) {
      TRANSLATE_X = 0;
      TRANSLATE_Y = 0;
      dim_raw = { (float)getWidth(), (float)getHeight() };
   }

   auto r_ego = my_r->findSectionWithEgoIfAny();
   std::vector<std::shared_ptr<RoadGraph>> all_nodes_ego_in_front{};

   all_nodes_ego_in_front.push_back(r_ego);

   for (const auto r_sub : all_nodes) {
      if (r_sub != r_ego) all_nodes_ego_in_front.push_back(r_sub);
   }

   const auto DRAW_STRAIGHT_ROAD_OR_CARS = [&](const RoadDrawingMode mode) {
      for (const auto r_sub : all_nodes_ego_in_front) {
         if (mode == RoadDrawingMode::road && r_sub->isGhost()
            || mode == RoadDrawingMode::ghosts_only && !r_sub->isGhost()) continue;

         const float section_max_lanes = r_sub->getMyRoad().getNumLanes();
         preserved_dimension_ = Vec2D{ dim_raw.x * section_max_lanes, dim_raw.y * section_max_lanes };

         const auto wrapper_trans_function = [this, section_max_lanes, r_sub, r_ego, old_trans, TRANSLATE_X, TRANSLATE_Y](const Vec3D& v_raw) -> Vec3D {
            const Vec2D origin{ r_sub->getOriginPoint().x, r_sub->getOriginPoint().y };// +(r_sub == r_ego && old_trans->is3D() && !r_sub->isGhost() ? (LANE_WIDTH * r_ego->my_road_.getEgo()->car_lane_ - section_max_lanes / 2) : 0) };
            const auto middle = plain_2d_translator_->translate({ origin.x, origin.y / LANE_WIDTH + (section_max_lanes / 2.0f) - 0.5f });
            Vec2D v{ plain_2d_translator_->translate({ v_raw.x + origin.x, v_raw.y + origin.y / LANE_WIDTH }) };
            v.rotate(r_sub->getAngle(), { middle.x, middle.y });
            auto res = plain_2d_translator_->reverseTranslate(v);

            return {
               res.x + TRANSLATE_X, 
               res.y + TRANSLATE_Y,
               v_raw.z };
            };

         const auto wrapper_reverse_trans_function = [this, section_max_lanes, r_sub, r_ego, old_trans, TRANSLATE_X, TRANSLATE_Y](const Vec3D& v_raw) -> Vec3D {
            const Vec2D origin{ r_sub->getOriginPoint().x, r_sub->getOriginPoint().y }; // -(r_sub == r_ego && old_trans->is3D() && !r_sub->isGhost() ? (LANE_WIDTH * r_ego->my_road_.getEgo()->car_lane_ - section_max_lanes / 2) : 0) };
            const auto middle = plain_2d_translator_->translate({ origin.x, origin.y / LANE_WIDTH + (section_max_lanes / 2.0f) - 0.5f });

            Vec2D v{
               v_raw.x - TRANSLATE_X,
               v_raw.y - TRANSLATE_Y,
            };

            v = plain_2d_translator_->translate(v);
            v.rotate(-r_sub->getAngle(), { middle.x, middle.y });
            auto res = plain_2d_translator_->reverseTranslate({ v.x, v.y });

            return { res.x - origin.x, res.y - origin.y / LANE_WIDTH, v_raw.z };
            };

         assert(Vec3D(0, 0, 0).isApproxEqual(wrapper_reverse_trans_function(wrapper_trans_function({ 0, 0, 0 }))));
         assert(Vec3D(0, 0, 0).isApproxEqual(wrapper_trans_function(wrapper_reverse_trans_function({ 0, 0, 0 }))));
         assert(Vec3D(10.3, 11, 5).isApproxEqual(wrapper_reverse_trans_function(wrapper_trans_function({ 10.3, 11, 5 }))));
         assert(Vec3D(10.3, 11, 5).isApproxEqual(wrapper_trans_function(wrapper_reverse_trans_function({ 10.3, 11, 5 }))));
         assert(Vec3D(.3, -11, 3).isApproxEqual(wrapper_reverse_trans_function(wrapper_trans_function({ .3, -11, 3 }))));
         assert(Vec3D(.3, -11, 3).isApproxEqual(wrapper_trans_function(wrapper_reverse_trans_function({ .3, -11, 3 }))));
         assert(Vec3D(-.3, -1.1, 0).isApproxEqual(wrapper_reverse_trans_function(wrapper_trans_function({ -.3, -1.1, 0 }))));
         assert(Vec3D(-.3, -1.1, 0).isApproxEqual(wrapper_trans_function(wrapper_reverse_trans_function({ -.3, -1.1, 0 }))));

         if (!infinite_road) {
            const auto wrapper_trans = std::make_shared<HighwayTranslatorWrapper>(
               old_trans,
               wrapper_trans_function,
               wrapper_reverse_trans_function);

            setTranslator(wrapper_trans);

            if (getHighwayTranslator()->is3D()) {
               const auto wrapper_trans_2d = std::make_shared<HighwayTranslatorWrapper>(
                  plain_2d_translator_,
                  wrapper_trans_function,
                  wrapper_reverse_trans_function);
               plain_2d_translator_wrapped_ = wrapper_trans_2d;
            }
         }

         r_sub->connectors_ = paintStraightRoadScene(
            r_sub->getMyRoad(),
            infinite_road,
            0,
            var_vals,
            print_agent_ids,
            infinite_road ? dim_raw : preserved_dimension_,
            mode == RoadDrawingMode::ghosts_only ? RoadDrawingMode::road : mode);

         if (!r_sub->isGhost()) writeAsciiText(10, -1, "Sec" + std::to_string(r_sub->getID()));
      }
      };

   DRAW_STRAIGHT_ROAD_OR_CARS(RoadDrawingMode::road);

   if (old_trans->is3D()) {
      setTranslator(old_trans);
   }
   else {
      setTranslator(std::make_shared<DefaultHighwayTranslator>());
   }

   // Draw crossings between sections.
   std::vector<Pol2D> additional_arrows{};
   //goto label;

   for (int i = 0; i <= 30; i++) {
      my_r->applyToMeAndAllMySuccessorsAndPredecessors([this, i, &dim_raw, &additional_arrows, old_trans](const std::shared_ptr<RoadGraph> r) -> void
         {
            for (const auto& r_succ : r->getSuccessors()) {
               for (const auto& A : r->connectors_) {
                  for (const auto& B : r_succ->connectors_) {
                     if (!r->isGhost() && !r_succ->isGhost() 
                        && A.id_ == i && A.id_ == B.id_ && A.side_ == ConnectorPolygonEnding::Side::drain && B.side_ == ConnectorPolygonEnding::Side::source) {
                        auto trans_a = A.my_trans_;
                        auto trans_b = B.my_trans_;

                        assert(*A.col_ == *B.col_);

                        Pol2D p{};
                        const auto norm_length_a = trans_a->translate({ 0, 0 }).distance(trans_a->translate({ 1, 0 }));
                        const auto norm_length_b = trans_b->translate({ 0, 0 }).distance(trans_b->translate({ 1, 0 }));
                        const auto a_connector_basepoint_translated = trans_a->translate(A.connector_.base_point_);
                        const auto a_connector_direction_translated = trans_a->translate(A.connector_.direction_);
                        const auto b_connector_basepoint_translated = trans_b->translate(B.connector_.base_point_);
                        const auto b_connector_direction_translated = trans_b->translate(B.connector_.direction_);
                        const auto thick_a = A.thick_ * norm_length_a;
                        const auto thick_b = B.thick_ * norm_length_b;

                        auto nice_points = bezier::getNiceBetweenPoints(a_connector_basepoint_translated, a_connector_direction_translated, b_connector_basepoint_translated, b_connector_direction_translated);

                        Vec2D between1 = nice_points[0];
                        Vec2D between1_dir = nice_points[1];
                        Vec2D between2 = nice_points[2];
                        Vec2D between2_dir = nice_points[3];

                        Vec2D between1_dir_ortho{ between1_dir };
                        between1_dir_ortho.ortho();
                        between1_dir_ortho.setLength(thick_a / 2);

                        Vec2D between2_dir_ortho{ between2_dir };
                        between2_dir_ortho.ortho();
                        between2_dir_ortho.setLength(thick_b / 2);

                        p.bezier(a_connector_basepoint_translated, between1, between2, b_connector_basepoint_translated, 0.01);
                        Pol2D arrow{};
                        arrow.createArrow(p, [p, thick_a, thick_b](const Vec2D& point_position, const int point_num) -> float
                           {
                              const float step{ (thick_b - thick_a) / p.points_.size() };
                              return thick_a + point_num * step;
                           }, {}, {});

                        if (*A.col_ == Color{ 0, 0, 0, 0 }) {
                           Pol2D arrow_square{};
                           arrow.add(*arrow.points_.begin());
                           arrow_square.createArrow(
                              arrow,
                              THICK * (old_trans->is3D() ? 2.1 : 0.97), // TODO: Remove these magic numbers
                              {},
                              {},
                              {},
                              {},
                              { 1, 1 },
                              { 1, 1 },
                              old_trans->is3D());

                           Vec2D point1a{ a_connector_basepoint_translated };
                           Vec2D point2a{ a_connector_basepoint_translated };
                           point1a.add(between1_dir_ortho);
                           point2a.sub(between1_dir_ortho);
                           Pol2D stop_line_pointsa{ point1a, point2a };
                           Pol2D stop_linea{};
                           stop_linea.createArrow(stop_line_pointsa, THICK * 2.1); // TODO: Remove these magic numbers

                           Vec2D point1b{ b_connector_basepoint_translated };
                           Vec2D point2b{ b_connector_basepoint_translated };
                           point1b.add(between2_dir_ortho);
                           point2b.sub(between2_dir_ortho);
                           Pol2D stop_line_pointsb{ point1b, point2b };
                           Pol2D stop_lineb{};
                           stop_lineb.createArrow(stop_line_pointsb, THICK * 2.1); // TODO: Remove these magic numbers

                           if (old_trans->is3D()) {
                              auto arrow_square_reverse = plain_2d_translator_->reverseTranslatePolygon(arrow_square);
                              auto stop_line_reversea = plain_2d_translator_->reverseTranslatePolygon(stop_linea);
                              auto stop_line_reverseb = plain_2d_translator_->reverseTranslatePolygon(stop_lineb);
                              fillPolygon(arrow_square_reverse, LANE_MARKER_COLOR);
                              fillPolygon(stop_line_reversea, LANE_MARKER_COLOR);
                              fillPolygon(stop_line_reverseb, LANE_MARKER_COLOR);
                           }
                           else {
                              // TODO: Get rid of this workaround.
                              for (int i = 1; i < arrow_square.points_.size(); i++) {
                                 if (arrow_square.points_[i].distance(arrow_square.points_[i - 1]) > 1000) {
                                    arrow_square.points_[i] = arrow_square.points_[i - 1];
                                 }
                              }
                              // EO TODO: Get rid of this workaround.

                              fillPolygon(arrow_square, LANE_MARKER_COLOR);
                           }

                           Pol2D p2{};
                           float begin = p.points_.size() / 4.0f;
                           float end = p.points_.size() - p.points_.size() / 4.0f;

                           for (int i = begin; i <= end; i++) {
                              p2.add(p.points_[i]);
                           }

                           auto vec = Pol2D::dashedArrow(p2, 2, 2.0f / 1500.0f * dim_raw.x * 3 * (old_trans->is3D() ? 2.1 : 1), {}, ARROW_END_PPT_STYLE_1, { 1, 1 }, { 2.5, 2.5 });

                           for (int i = 0; i < vec.size(); i++) {
                              if (i % 2 || i == vec.size() - 1)
                                 additional_arrows.push_back(vec[i]);
                           }
                        }
                        else {
                           if (old_trans->is3D()) {
                              auto arrow_reverse = plain_2d_translator_->reverseTranslatePolygon(arrow);
                              fillPolygon(arrow_reverse, *A.col_);
                           }
                           else {
                              fillPolygon(arrow, *A.col_);
                           }
                        }
                        //drawPolygon(arrow, BLACK, true, true, true);
                     }
                  }
               }
            }
         });
   }

   constexpr bool DRAW_DIRECTION_ARROWS_ON_JUNCTIONS = { false };

   if (DRAW_DIRECTION_ARROWS_ON_JUNCTIONS) {
      for (const auto& a : additional_arrows) {
         if (!a.points_.empty()) {
            if (old_trans->is3D()) {
               auto a_reverse = plain_2d_translator_->reverseTranslatePolygon(a);
               fillPolygon(a_reverse, DARK_GREY);
            }
            else {
               fillPolygon(a, DARK_GREY);
            }
         }
      }
   }

   label:
   setTranslator(old_trans);
   //DRAW_STRAIGHT_ROAD_OR_CARS(RoadDrawingMode::ghosts_only); // For debugging.
   DRAW_STRAIGHT_ROAD_OR_CARS(RoadDrawingMode::cars);
   setTranslator(old_trans);
}
