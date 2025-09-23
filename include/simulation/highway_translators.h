//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#pragma once
#include "geometry/images.h"
#include "geometry/line_segment_3d.h"
#include "geometry/line_3d.h"
#include "geometry/plane_3d.h"
#include <cmath>

namespace vfm {

static std::map<int, std::vector<double>> preprocessed_z_to_y_pos_sum_{};
static constexpr float TOP_FACTOR = 8;
static constexpr float V_POINT_FACTOR = TOP_FACTOR; // Sets vpoint to edge of earth part when equal to TOP_FACTOR (above, when larger). 

class HighwayTranslator : public VisTranslator
{
public:
   HighwayTranslator(const bool mirrored) : mirrored_{ mirrored } {}

   virtual bool is3D() const = 0;
   bool isMirrored() const
   {
      return mirrored_;
   }

   virtual inline void setHighwayData(
      const float factor,
      const float real_width,
      const float real_height,
      const float ego_offset_x,
      const float street_top,
      const int min_lane,
      const int max_lane,
      const float lw,
      const float ego_lane,
      const Vec2D& v_point)
   {
      factor_ = factor;
      real_width_ = real_width;
      real_height_ = real_height;
      ego_offset_x_ = ego_offset_x;
      street_top_ = street_top;
      min_lane_ = min_lane;
      max_lane_ = max_lane;
      lw_ = lw;
      ego_lane_ = ego_lane;
      v_point_ = v_point;
   }

public: // TODO: Make protected again (or private!).
   float factor_{};
   float real_width_{};
   float real_height_{};
   float ego_offset_x_{};
   float street_top_{};
   int min_lane_{};
   int max_lane_{};
   float lw_{};
   float ego_lane_{};
   bool mirrored_{};
   Vec2D v_point_{};
};

/// This is the "highway" version of the default translator which just maps each
/// point (projected to the floor) on itself (identity relation). Note that this will most likely not be
/// what you want if you're coming from the MC context. Use the "Plain" translators, then.
class DefaultHighwayTranslator : public HighwayTranslator
{
public:
   DefaultHighwayTranslator() : HighwayTranslator(false) {}

   inline bool is3D() const override
   {
      return false;
   }

private:
   DefaultTranslator regular_translator_{};

   inline Vec2D translateCore(const Vec3D& point) override { return regular_translator_.translate(point); }
   inline Vec3D reverseTranslateCore(const Vec2D& point) override { return regular_translator_.reverseTranslate(point); }
};

class Plain2DTranslator : public HighwayTranslator
{
public:
   Plain2DTranslator() : HighwayTranslator(false) {}

   inline bool is3D() const override
   {
      return false;
   }

private:
   inline Vec2D translateCore(const Vec3D& point) override
   {
      return { point.x * factor_ + real_width_ / 2,
               street_top_ + (point.y + ego_lane_ - min_lane_) * lw_ + lw_ / 2 };
   }

   inline Vec3D reverseTranslateCore(const Vec2D& point) override {
      return { (point.x - real_width_ / 2) / factor_,
               (point.y - street_top_ - lw_ / 2) / lw_ + min_lane_ - ego_lane_,
               0 };
   }
};

class VanishingPointTranslator : public HighwayTranslator
{
public:
   VanishingPointTranslator(const bool mirrored) : HighwayTranslator(mirrored) {}

   inline bool is3D() const override
   {
      return true;
   }

private:
   static constexpr double FX0{ 1 };


   inline float getYFromZAtZeroElevation(const double z_pos_raw) const
   {
      // 1000 is probably a much too high value (corresponds to 
      // rasterization based on 1mm, divided by whatever comes 
      // out of the divisor of z_pos, of distance in Z direction), but doesn't hurt.
      constexpr double SMOOTHNESS_FACTOR{ 1000.0 };

      double z_pos{ z_pos_raw * real_height_ / 20 };

      const int zi{ (int)std::round(z_pos * SMOOTHNESS_FACTOR) };
      const double full_range{ real_height_ - v_point_.y };
      const int fri{ (int)full_range };
      preprocessed_z_to_y_pos_sum_.insert({ fri, { FX0 / SMOOTHNESS_FACTOR } });
      auto& preprocessed_z_to_y_pos_sum{ preprocessed_z_to_y_pos_sum_[fri] };

      while (preprocessed_z_to_y_pos_sum.size() <= zi) {
         const double sum{ preprocessed_z_to_y_pos_sum[preprocessed_z_to_y_pos_sum.size() - 1] };
         const double fxn{ (1 - sum / full_range) * preprocessed_z_to_y_pos_sum[0] };
         preprocessed_z_to_y_pos_sum.push_back(fxn + sum);
      }

      return real_height_ - preprocessed_z_to_y_pos_sum[zi];

      // Fairly well working closed formula:
      // return v_point_.y + ((real_height_ - v_point_.y) / (z_pos * std::sqrt(z_pos) / 300 + 1));
   }

   float getYFromZAtElevation(const float z_pos, const float height) const
   {
      static const float EPSILON{ 0.001 };
      const float z_pos_zero{ getYFromZAtZeroElevation(z_pos) };

      if (abs(height) <= EPSILON) return z_pos_zero;

      const float X_POS{ v_point_.x - 100 };

      Vec2D point1{ X_POS, 0 };
      Vec2D point2{ X_POS, height };

      Line2D<float> l_at_z_pos_h{ Line2D<float>{ {0, z_pos_zero}, { 1, 0 } } };
      LineSegment2D<float> v_line1{ point1, v_point_ };
      LineSegment2D<float> v_line2{ point2, v_point_ };
      Vec2D point1_z{ *l_at_z_pos_h.intersect(Line2D<float>(v_line1)) };
      Line2D<float> l_at_z_pos_v{ Line2D<float>{ point1_z, { 0, 1 } } };

      return l_at_z_pos_v.intersect(Line2D<float>(v_line2))->y;
   }

   inline Vec2D translateCore(const Vec3D& point_raw) override
   {
      constexpr float REFERENCE_LANE_NUM{ 3 };
      const float lane_w_unscaled_ = (real_width_ - 60) / REFERENCE_LANE_NUM;

      auto point{ point_raw.projectToXY() };
      //point.rotate(getPerspective()->camera_alpha_, { 0, 0 });

      float point_x{ point.x };
      float point_y{ point.y };
      float point_z{ point_raw.z };

      if (mirrored_) {
         point_x = -point_x;
         point_y += 2 * 0.038; // TODO: What is this 0.038 number??
      }

      float real_x{ point_x };
      
      if (real_x < 0) real_x = 0;

      float z_pos1 = getYFromZAtZeroElevation(real_x);
      float height = point_z != 0 ? (getYFromZAtElevation(real_x, point_z) - z_pos1) : 0;

      height *= v_point_.y / 3;

      auto l_at_z_pos1 = Line2D<float>(Line2D<float>{ {0, z_pos1}, { 1, 0 } });
      float actual_x = (point_y + REFERENCE_LANE_NUM - 1.5) * lane_w_unscaled_;

      auto projected_point = l_at_z_pos1.intersect(
         Line2D<float>(LineSegment2D<float>({ { actual_x, real_height_ }, v_point_ })));

      if (projected_point) {
         return { projected_point->x, projected_point->y + height };
      }
      else {
         addError("Intersection does not exist for point '" + Vec2Df{ point_x, point_y }.serialize() + "' to Plain3DTranslator.");
      }

      return { 0, 0 };
   }

   inline Vec3D reverseTranslateCore(const Vec2D& point) override {
      addFatalError("Reverse translation not implemented for VanishingPointTranslator.");
      return { 0, 0, 0 };
   }
};

class Plain3DTranslator : public HighwayTranslator // TODO: Only inherit from VisTranslator. We don't need to be on a highway.
{
public:
   Plain3DTranslator(const bool mirrored) : HighwayTranslator(mirrored) {}

   inline bool is3D() const override
   {
      return true;
   }

private:
   inline Vec3D getPointInCameraCoordinates(const Vec3D& point_raw) const
   {
      const auto perspective{ getPerspective() };
      const Vec3D point{ point_raw.z, point_raw.y * (mirrored_ ? -1 : 1), point_raw.x };

      // Cf. https://en.wikipedia.org/w/index.php?title=3D_projection&oldid=1190317936#Mathematical_formula
      const double x{ point.x - perspective->getCameraX() };
      const double y{ point.y - perspective->getCameraY() };
      const double z{ point.z - perspective->getCameraZ() };
      const double sx{ perspective->getRotationCameraXSin() };
      const double sy{ perspective->getRotationCameraYSin() };
      const double sz{ perspective->getRotationCameraZSin() };
      const double cx{ perspective->getRotationCameraXCos() };
      const double cy{ perspective->getRotationCameraYCos() };
      const double cz{ perspective->getRotationCameraZCos() };

      const double dz{ cx * (cy * z + sy * (sz * y + cz * x)) - sx * (cz * y - sz * x) };
      const double dx{ cy * (sz * y + cz * x) - sy * z };
      const double dy{ sx * (cy * z + sy * (sz * y + cz * x)) + cx * (cz * y - sz * x) };

      return { (float)dx, (float)dy, (float)dz };
   }

   inline Vec2D projectPointInCameraPerspectiveTo2D(const Vec3D& d) const
   {
      const auto perspective{ getPerspective() };

      const double ex{ perspective->getDisplayWindowX() };
      const double ey{ perspective->getDisplayWindowY() };
      const double ez{ perspective->getDisplayWindowZ() };

      double bx{ ez / d.z * d.x + ex };
      double by{ ez / d.z * d.y + ey };

      bx *= real_width_;
      //by += .06; // TODO!
      by *= real_height_;

      return { (float)bx, (float)by };
   }

   inline Vec2D translateCore(const Vec3D& point_raw) override
   {
      const auto d{ getPointInCameraCoordinates(point_raw) };
      if (d.z >= 0) { // Check if point it in front of the camera.
         return projectPointInCameraPerspectiveTo2D(d);
      } else { // Return NAN if behind the camera. For polygons partially behind camera use translatePolygon(...).
         return { std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN() };
      }
   }

   inline Vec3D reverseTranslateCore(const Vec2D& point) override 
   {
      addFatalError("Reverse translation not implemented for Plain3DTranslator.");
      return { 0, 0, 0 };
   }

   /// <summary>
   /// Adds for two points P, Q, one of which is is behind, the other in front of the camera, the intersection point
   /// of the line through P and Q to the resulting polygon. This accomplishes the clipping at the visible plane.
   /// </summary>
   /// <param name="last_point_in_camera_coordinates">One of the points P, Q, can be in front of or behind the camera.</param>
   /// <param name="point_in_camera_coordinates">One of the points P, Q, can be in front of or behind the camera.</param>
   /// <param name="result">The resulting polygon which the intersection will be added to.</param>
   inline void addIntersectionPointToResult(const Vec3D& last_point_in_camera_coordinates, const Vec3D& point_in_camera_coordinates, Pol2D& result) const
   {
      Pln3D camera_plane{ getPerspective()->getDisplayPlane() };
      Lin3D current_line{ LinSeg3D{ last_point_in_camera_coordinates, point_in_camera_coordinates } };
      auto intersection{ camera_plane.intersect(current_line) };

      if (intersection) {
         result.add(projectPointInCameraPerspectiveTo2D(*intersection));
      }
      else {
         Failable::getSingleton()->addFatalError("No intersection"); // TODO: Just for now, remove later.
      }
   }

   inline Pol2D translatePolygonCore(const Pol3D& pol) override
   { // Clips polygons at visible plane if they are partially behind and partially in front of the camera.
      Pol2D result{};
      bool last_behind_camera{ false };
      bool current_behind_camera{ false };
      bool yes{};
      Vec3D current_point{ *(pol.points_.end() - 1) };
      Vec3D last_point{};
      Vec3D current_point_in_camera_coordinates{};
      Vec3D last_point_in_camera_coordinates{};
      int first{ 0 };
      bool found{ false };
      bool firstit{ true };

      for (; first < pol.points_.size(); first++) { // Find first point that is in front of the camera.
         last_point = current_point;
         current_point = pol.points_.at(first);

         if (getPointInCameraCoordinates(current_point).z >= 0) {
            found = true;
            break;
         }
      }

      if (!found) return {}; // No point is in front of the camera.

      last_point_in_camera_coordinates = getPointInCameraCoordinates(last_point);

      for (int i = first; i != first || firstit; i = (i + 1) % pol.points_.size()) {
         current_point_in_camera_coordinates = getPointInCameraCoordinates(pol.points_.at(i));
         current_behind_camera = current_point_in_camera_coordinates.z <= 0;
         last_behind_camera = last_point_in_camera_coordinates.z <= 0;

         if (current_behind_camera) {
            if (!last_behind_camera) { // Current point behind the camera, but the last was still fine.
               addIntersectionPointToResult(last_point_in_camera_coordinates, current_point_in_camera_coordinates, result);
               yes = true;
            }
            else { 
               // We are in a sequence of behind-camera points. Ignore these.
            }
         } else {
            if (last_behind_camera) {        // Current point in front of camera, but last was behind.
               addIntersectionPointToResult(last_point_in_camera_coordinates, current_point_in_camera_coordinates, result);
               yes = true;
            }

            result.add(projectPointInCameraPerspectiveTo2D(current_point_in_camera_coordinates));
         }

         last_point_in_camera_coordinates = current_point_in_camera_coordinates;
         firstit = false;
      }

      return result;
   }
};

/// <summary>
/// Takes another translator trans as base, but processes each point with a custom wrapper_function
/// before passing it to trans.
/// </summary>
class HighwayTranslatorWrapper : public HighwayTranslator {
public:
   inline HighwayTranslatorWrapper(
      const std::shared_ptr<HighwayTranslator> trans,
      const std::function<Vec3D(const Vec3D&)> wrapper_function,
      const std::function<Vec3D(const Vec3D&)> wrapper_function_reverse) : 
      base_translator_(trans),
      wrapper_function_(wrapper_function),
      wrapper_function_reverse_(wrapper_function_reverse),
      HighwayTranslator(trans->isMirrored())
   {
      setPerspective(base_translator_->getPerspective());
      setHighwayData(
         base_translator_->factor_,
         base_translator_->real_width_,
         base_translator_->real_height_,
         base_translator_->ego_offset_x_,
         base_translator_->street_top_,
         base_translator_->min_lane_,
         base_translator_->max_lane_,
         base_translator_->lw_,
         base_translator_->ego_lane_,
         base_translator_->v_point_);
   }

   Vec2D translateCore(const Vec3D& point) override
   {
      auto wrapped = wrapper_function_(point);
      auto wrapped_res = base_translator_->translateCore(wrapped);
      return wrapped_res;
   }

   Vec3D reverseTranslateCore(const Vec2Df& point) override
   {
      auto res = base_translator_->reverseTranslateCore(point);
      auto res_wrapped = wrapper_function_reverse_(res);
      return res_wrapped;
   }

   inline Pol2D translatePolygonCore(const Pol3D& pol_raw) override
   {
      Pol3D pol{};

      for (const auto& p : pol_raw.points_) {
         pol.add(wrapper_function_(p));
      }

      return base_translator_->translatePolygonCore(pol);
   }

   bool is3D() const override
   {
      return base_translator_->is3D();
   }

   inline void setHighwayData(
      const float factor,
      const float real_width,
      const float real_height,
      const float ego_offset_x,
      const float street_top,
      const int   min_lane,
      const int   max_lane,
      const float lw,
      const float ego_lane,
      const Vec2D& v_point) override
   {
      HighwayTranslator::setHighwayData(
         factor,
         real_width,
         real_height,
         ego_offset_x,
         street_top,
         min_lane,
         max_lane,
         lw,
         ego_lane,
         v_point);

      base_translator_->setHighwayData(
         factor,
         real_width,
         real_height,
         ego_offset_x,
         street_top,
         min_lane,
         max_lane,
         lw,
         ego_lane,
         v_point);
   }

   inline void setPerspective(const std::shared_ptr<VisPerspective> perspective) override
   {
      HighwayTranslator::setPerspective(perspective);
      base_translator_->setPerspective(perspective);
   }

private:
   std::shared_ptr<HighwayTranslator> base_translator_{};
   std::function<Vec3D(const Vec3D&)> wrapper_function_{};
   std::function<Vec3D(const Vec3D&)> wrapper_function_reverse_{};
};

} // vfm
