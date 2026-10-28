//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "geometry/polygon_2d.h"
#include "geometry/vector_2d.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace vfm::bezier {

static std::array<Vec2D, 4> connectorBezier(
   const Vec2D& source, const Vec2D& source_direction,
   const Vec2D& target, const Vec2D& target_direction, const float lane_offset = 0)
{
   Vec2D source_normal{ source_direction };
   Vec2D target_normal{ target_direction };
   source_normal.ortho();
   target_normal.ortho();
   source_normal.setLength(lane_offset);
   target_normal.setLength(lane_offset);
   const Vec2D start{ source + source_normal };
   const Vec2D end{ target + target_normal };
   const float handle_length{ start.distance(end) / 3 };
   Vec2D source_handle{ source_direction };
   Vec2D target_handle{ target_direction };
   source_handle.setLength(handle_length);
   target_handle.setLength(handle_length);
   return { start, start + source_handle, end - target_handle, end };
}

namespace detail {

static bool curvatureBoundHolds(const std::array<Vec2Dd, 3>& velocity,
   const std::array<Vec2Dd, 2>& acceleration, const std::array<double, 3>& velocity_error,
   const std::array<double, 2>& acceleration_error, const double radius, const int subdivisions_left)
{
   const auto cross = [](const Vec2Dd& first, const Vec2Dd& second) {
      return first.x * second.y - first.y * second.x;
   };
   const std::array<double, 4> numerator{
      cross(velocity[0], acceleration[0]),
      (2 * cross(velocity[1], acceleration[0]) + cross(velocity[0], acceleration[1])) / 3,
      (cross(velocity[2], acceleration[0]) + 2 * cross(velocity[1], acceleration[1])) / 3,
      cross(velocity[2], acceleration[1]) };
   const auto cross_error = [&](const size_t velocity_index, const size_t acceleration_index) {
      const auto& speed{ velocity[velocity_index] };
      const auto& change{ acceleration[acceleration_index] };
      return std::hypot(speed.x, speed.y) * acceleration_error[acceleration_index]
         + std::hypot(change.x, change.y) * velocity_error[velocity_index]
         + velocity_error[velocity_index] * acceleration_error[acceleration_index];
   };
   const std::array<double, 4> numerator_error{ cross_error(0, 0),
      (2 * cross_error(1, 0) + cross_error(0, 1)) / 3,
      (cross_error(2, 0) + 2 * cross_error(1, 1)) / 3, cross_error(2, 1) };
   const double min_x{ std::max({ 0.0,
      std::min({ velocity[0].x - velocity_error[0], velocity[1].x - velocity_error[1], velocity[2].x - velocity_error[2] }),
      -std::max({ velocity[0].x + velocity_error[0], velocity[1].x + velocity_error[1], velocity[2].x + velocity_error[2] }) }) };
   const double min_y{ std::max({ 0.0,
      std::min({ velocity[0].y - velocity_error[0], velocity[1].y - velocity_error[1], velocity[2].y - velocity_error[2] }),
      -std::max({ velocity[0].y + velocity_error[0], velocity[1].y + velocity_error[1], velocity[2].y + velocity_error[2] }) }) };
   const double speed_squared_lower{ min_x * min_x + min_y * min_y };
   double numerator_upper{};
   double velocity_upper{};
   double acceleration_upper{};
   for (size_t index = 0; index < numerator.size(); ++index)
      numerator_upper = std::max(numerator_upper, std::abs(numerator[index]) + numerator_error[index]);
   for (const auto& value : velocity) velocity_upper = std::max(velocity_upper, std::hypot(value.x, value.y));
   for (const auto& value : acceleration) acceleration_upper = std::max(acceleration_upper, std::hypot(value.x, value.y));
   const double roundoff{ 64 * std::numeric_limits<double>::epsilon() };
   const double speed_lower{ std::max(0.0, std::sqrt(speed_squared_lower) - roundoff * velocity_upper) };
   if (speed_lower > 0 && (numerator_upper + roundoff * velocity_upper * acceleration_upper) * radius
      <= speed_lower * speed_lower * speed_lower) return true;

   const Vec2Dd velocity_left{ (velocity[0] + velocity[1]) / 2 };
   const Vec2Dd velocity_right{ (velocity[1] + velocity[2]) / 2 };
   const Vec2Dd velocity_middle{ (velocity_left + velocity_right) / 2 };
   const Vec2Dd acceleration_middle{ (acceleration[0] + acceleration[1]) / 2 };
   for (const auto& sample : { std::pair{ velocity[0], acceleration[0] },
      std::pair{ velocity_middle, acceleration_middle }, std::pair{ velocity[2], acceleration[1] } }) {
      const double speed{ std::hypot(sample.first.x, sample.first.y) };
      if (!(speed > 0) || std::abs(cross(sample.first, sample.second)) * radius > speed * speed * speed) return false;
   }
   if (subdivisions_left == 0) return false;
      const double velocity_error_left{ (velocity_error[0] + velocity_error[1]) / 2 };
      const double velocity_error_right{ (velocity_error[1] + velocity_error[2]) / 2 };
      const double velocity_error_middle{ (velocity_error_left + velocity_error_right) / 2 };
      const double acceleration_error_middle{ (acceleration_error[0] + acceleration_error[1]) / 2 };
   return curvatureBoundHolds({ velocity[0], velocity_left, velocity_middle },
            { acceleration[0], acceleration_middle },
            { velocity_error[0], velocity_error_left, velocity_error_middle },
            { acceleration_error[0], acceleration_error_middle }, radius, subdivisions_left - 1)
      && curvatureBoundHolds({ velocity_middle, velocity_right, velocity[2] },
            { acceleration_middle, acceleration[1] },
            { velocity_error_middle, velocity_error_right, velocity_error[2] },
            { acceleration_error_middle, acceleration_error[1] }, radius, subdivisions_left - 1);
}

}

static bool hasMinimumRadius(const std::array<Vec2D, 4>& curve, const double radius, const double endpoint_error = 0)
{
   if (radius <= 0) return true;
   if (!std::isfinite(radius) || !std::isfinite(endpoint_error) || endpoint_error < 0) return false;
   std::array<Vec2Dd, 3> velocity;
   for (size_t index = 0; index < velocity.size(); ++index) {
      velocity[index] = { 3 * (static_cast<double>(curve[index + 1].x) - curve[index].x),
         3 * (static_cast<double>(curve[index + 1].y) - curve[index].y) };
      if (!std::isfinite(velocity[index].x) || !std::isfinite(velocity[index].y)) return false;
   }
   return detail::curvatureBoundHolds(velocity,
      { (velocity[1] - velocity[0]) * 2, (velocity[2] - velocity[1]) * 2 },
      { endpoint_error, 5 * endpoint_error, endpoint_error },
      { 12 * endpoint_error, 12 * endpoint_error }, radius, 18);
}

// Function to calculate the point on a cubic Bezier curve
static Vec2D cubicBezier(float t, const Vec2D& p0, const Vec2D& p1, const Vec2D& p2, const Vec2D& p3) {
   return p0 * pow(1 - t, 3) + p1 * (3 * pow(1 - t, 2) * t) + p2 * (3 * (1 - t) * pow(t, 2)) + p3 * pow(t, 3);
}

// Function to calculate the derivative of a cubic Bezier curve
static Vec2D B_prime(float t, const Vec2D& p0, const Vec2D& p1, const Vec2D& p2, const Vec2D& p3) {
   return (p1 - p0) * (3 * pow(1 - t, 2)) + (p2 - p1) * (6 * (1 - t) * t) + (p3 - p2) * (3 * pow(t, 2));
}

// Maximum unsigned curvature of a cubic Bezier curve, sampled at n + 1 parameter values; infinite at a cusp.
static float maxCurvature(const Vec2D& p0, const Vec2D& p1, const Vec2D& p2, const Vec2D& p3, int n = 500) {
   float k_max = 0;
   for (int i = 0; i <= n; ++i) {
      const float t = static_cast<float>(i) / n;
      const Vec2D d1 = B_prime(t, p0, p1, p2, p3);
      const Vec2D d2 = (p2 - p1 * 2 + p0) * (6 * (1 - t)) + (p3 - p2 * 2 + p1) * (6 * t);
      const float speed = d1.length();
      if (speed < 1e-6f) return std::numeric_limits<float>::infinity();
      k_max = std::max(k_max, std::abs(d1.x * d2.y - d1.y * d2.x) / (speed * speed * speed));
   }
   return k_max;
}

// Function to approximate the arc length from s to B(t) using numerical integration (Trapezoidal Rule)
static float arcLength(float t, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e) {
   int n = 100; // Number of segments for integration (adjust as needed)
   float dt = t / n;
   float length = 0.0;
   for (int i = 0; i < n; ++i) {
      float t1 = i * dt;
      float t2 = (i + 1) * dt;
      Vec2D p1 = B_prime(t1, s, v, w, e);  // Evaluate derivative at t1
      Vec2D p2 = B_prime(t2, s, v, w, e);  // Evaluate derivative at t2
      length += 0.5 * (p1.length() + p2.length()) * dt; // Trapezoidal rule
   }
   return length;
}

// Function to approximate the arc length from s to B(t) using numerical integration (Trapezoidal Rule); omit the square for vector length calculation
static float arcLengthSquare(float t, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e) {
   int n = 100; // Number of segments for integration (adjust as needed)
   float dt = t / n;
   float length = 0.0;
   for (int i = 0; i < n; ++i) {
      float t1 = i * dt;
      float t2 = (i + 1) * dt;
      Vec2D p1 = B_prime(t1, s, v, w, e);  // Evaluate derivative at t1
      Vec2D p2 = B_prime(t2, s, v, w, e);  // Evaluate derivative at t2
      length += 0.5 * (p1.lengthSquare() + p2.lengthSquare()) * dt; // Trapezoidal rule
   }
   return length;
}

// Function to find the parameter t such that arcLength(t) ≈ l using bisection
static float findT(float l, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e, float tolerance = 1e-6) {
   float a = 0.0;  // Lower bound for t
   float b = 1.0;  // Upper bound for t
   while ((b - a) > tolerance) {
      float mid = (a + b) / 2.0;
      if (arcLength(mid, s, v, w, e) < l) {
         a = mid;
      }
      else {
         b = mid;
      }
   }
   return (a + b) / 2.0;  // Return the midpoint as an approximation
}

// Function to calculate the point at a given length (relative to the full arc length) along a cubic Bezier curve
static Vec2D pointAtRatio(float l, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e) {
   assert(0 <= l && l <= 1);
   float totalLength = arcLength(1.0, s, v, w, e);
   float t = findT(l * totalLength, s, v, w, e);
   return cubicBezier(t, s, v, w, e);
}

static std::vector<Vec2D> getNiceBetweenPoints(const Vec2D& point_orig, const Vec2D& dir_orig, const Vec2D& point_targ, const Vec2D& dir_targ)
{
   Vec2D between1 = point_orig;
   Vec2D between1_dir = point_orig;
   between1_dir.sub(dir_orig);

   between1_dir.setLength(point_orig.distance(point_targ) / 3);
   between1.add(between1_dir);
   Vec2D between2 = point_targ;
   Vec2D between2_dir = point_targ;
   between2_dir.sub(dir_targ);

   between2_dir.setLength(point_orig.distance(point_targ) / 3);
   between2.add(between2_dir);

   return { between1, between1_dir, between2, between2_dir };
}

} // vfm::bezier
