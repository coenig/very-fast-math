//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "geometry/polygon_2d.h"
#include "geometry/vector_2d.h"
#include <cmath>

namespace vfm::bezier {

// Function to calculate the point on a cubic Bezier curve
static Vec2D cubicBezier(float t, const Vec2D& p0, const Vec2D& p1, const Vec2D& p2, const Vec2D& p3) {
   return p0 * pow(1 - t, 3) + p1 * (3 * pow(1 - t, 2) * t) + p2 * (3 * (1 - t) * pow(t, 2)) + p3 * pow(t, 3);
}

// Function to calculate the derivative of a cubic Bezier curve
static Vec2D B_prime(float t, const Vec2D& p0, const Vec2D& p1, const Vec2D& p2, const Vec2D& p3) {
   return (p1 - p0) * (3 * pow(1 - t, 2)) + (p2 - p1) * (6 * (1 - t) * t) + (p3 - p2) * (3 * pow(t, 2));
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
