//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "geometry/vector_2d.h"
#include <cmath>

namespace vfm::bezier {

// Function to calculate the point on a cubic Bezier curve
Vec2D cubicBezier(float t, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e) {
   return s * pow(1 - t, 3) + v * (3 * pow(1 - t, 2) * t) + w * (3 * (1 - t) * pow(t, 2)) + e * pow(t, 3);
}

// Function to calculate the derivative of a cubic Bezier curve
Vec2D B_prime(float t, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e) {
   return (v - s) * (3 * pow(1 - t, 2)) + (w - v) * (6 * (1 - t) * t) + (e - w) * (3 * pow(t, 2));
}

// Function to calculate the magnitude of a vector
float magnitude(const Vec2D& vector) {
   return vector.length();
}

// Function to approximate the arc length from s to B(t) using numerical integration (Trapezoidal Rule)
float arcLength(float t, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e) {
   int n = 100; // Number of segments for integration (adjust as needed)
   float dt = t / n;
   float length = 0.0;
   for (int i = 0; i < n; ++i) {
      float t1 = i * dt;
      float t2 = (i + 1) * dt;
      Vec2D p1 = B_prime(t1, s, v, w, e);  // Evaluate derivative at t1
      Vec2D p2 = B_prime(t2, s, v, w, e);  // Evaluate derivative at t2
      length += 0.5 * (magnitude(p1) + magnitude(p2)) * dt; // Trapezoidal rule
   }
   return length;
}

// Function to find the parameter t such that arcLength(t) ≈ l using bisection
float findT(float l, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e, float tolerance = 1e-6) {
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
Vec2D pointAtRatio(float l, const Vec2D& s, const Vec2D& v, const Vec2D& w, const Vec2D& e) {
   assert(0 <= l && l <= 1);
   float totalLength = arcLength(1.0, s, v, w, e);
   float t = findT(l * totalLength, s, v, w, e);
   return cubicBezier(t, s, v, w, e);
}
} // vfm::bezier