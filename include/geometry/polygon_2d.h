//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "abstract_object.h"
#include "circle_2d.h"
#include "line_2d.h"
#include "line_segment_2d.h"
#include "rectangle_2d.h"
#include "vector_2d.h"
#include "failable.h"
#include <initializer_list>
#include <cmath>
#include <math.h>
#include <vector>
#include <cassert>
#include <string>
#include <memory>
#include <iostream>
#include <functional>


namespace vfm {

template<class NumType>
class Polygon2D : public Object2D<NumType> {
public:
   using Point = typename Object2D<NumType>::Point;
   using PointVec = typename Object2D<NumType>::PointVec;

   std::vector<Vector2D<NumType>> points_; /// Don't change from outside (just read) to avoid inconsistencies with bounding box.

   Polygon2D() {}
   Polygon2D(const std::vector<Vector2D<NumType>>& p);
   Polygon2D(const Polygon2D<NumType>& other);
   Polygon2D(std::initializer_list<Vector2D<NumType>> vecs);

   /// \brief Creates a plain copy of the polygon.
   Polygon2D<NumType> toPol() const override;
   std::string serialize() const override;

   void add(const Vector2D<NumType>& p);
   void addAll(const Polygon2D<NumType>& p);
   void add(const int position, const Vector2D<NumType>& p);
   std::shared_ptr<Rectangle2D<NumType>> getBoundingBox() const;

   /// \brief Calculates all intersection points of the line segment with this.
   std::vector<Vector2D<NumType>> intersect(const LineSegment2D<NumType>& s1) const;

   /// \brief Calculates all intersection points of this Polygon with another.
   std::vector<Vector2D<NumType>> intersect(const Polygon2D<NumType>& other) const;

   /// \brief Similar to "intersect", but faster. Only difference is that it catches
   /// the case that the whole line segment is within the polygon (which is more often
   /// what you'll want, probably).
   bool hasCommonPointWith(const LineSegment2D<NumType>& s1) const;

   /// \brief Similar to "intersect", but faster. Only difference is that it catches
   /// the case that one polygon is completely within the other (which is more often
   /// what you'll want, probably).
   bool hasCommonPointWith(const Polygon2D<NumType>& other) const;

   void translate(const Vector2D<NumType>& vek);
   void scale(const NumType scal);
   void scale(const NumType scal, const Vector2D<NumType>& center);
   void scale(const Vector2D<NumType>& scal);
   void scale(const Vector2D<NumType>& scal, const Vector2D<NumType>& center);
   void rotate(const float angle_rad);
   void rotate(const float angle_rad, const Vector2D<NumType>& center);

   /// \brief Rearranges all polygon points such that adjacent
   ///  points all have equal distance, which is the average distance
   ///  in the original polygon. The new polygon is fitted "fairly close" to 
   ///  the original polygon's shape, although no guarantees are given to the 
   ///  magnitude of the change. The new polygon has either the same number of 
   ///  points as the original or one more.
   Polygon2D<NumType> normalize(const bool close = false) const;

   bool isPointWithin(const Vector2D<NumType>& punkt) const;
   Vector2D<NumType> centerPoint();
   
   /// \brief Returns a polygon in counterclockwise ordering of the vertices for all star-shaped polygons 
   /// (http://en.wikipedia.org/wiki/Star-shaped_polygon). For other polygons the only assurance is that 
   /// the step from the first vertex to the second is in counterclockwise ordering.
   /// this is returned if the polygon is already ordered counterclockwise.
   Polygon2D<NumType> getCounterClockwiseOrderedIfStarShaped();

   /// \brief The same polygon as this except for a reverted ordering of the vertices.
   Polygon2D<NumType> getReverted() const;

   void translateRootPointToMiddle();
   void insertNodesOnExistingEdge(const int edg1Num, const float nodesToInsert);

   /// \brief Inserts the requested number of nodes on any edge of the polygon. Therefore, the number of nodes
   ///  in the polygon gets multiplied by that number. The shape of the polygon stays unchanged. The new nodes
   ///  of any edge are inserted with a constant distance to each other and the two original nodes of the edge.
   void insertNodesOnExistingEdges(const int nodesToInsert, const bool close_polygon = true);

   /// \brief Returns iff this polygon is convex by checking if all angles are smaller or equal to 180 degrees.
   bool isConvex() const;

   /// Retrieves a polygon that is the intersection of an arbitrary polygon (the "subject" polygon) 
   /// and a convex polygon (the "clip" polygon). Takes two polygons (this and a parameter), 
   /// one of which has to be convex. Both this and the parameter polygon are not changed in the process, 
   /// the resulting intersection polygon is returned. 
   /// Cf. https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
   ///
   /// NOTE: The convex polygon (which is also star-shaped) needs to have all points in counter-clockwise order.
   Polygon2D<NumType> clipPolygonsOneConvex(const Polygon2D<NumType>& p2) const;

   /// \brief Adds a bezier curve through the points (p1, p2, p3, p4). Point distance is controlled via step in [0, 1].
   Polygon2D<NumType> bezier(const Vector2D<NumType>& p0, const Vector2D<NumType>& p1, const Vector2D<NumType>& p2, const Vector2D<NumType>& p3, const float step = 0.01);

   /// \brief Adds a bezier curve through arbitrary many points with associated direction vectors. 
   /// Point distance is controlled via step in [0, 1].
   Polygon2D<NumType> multiBezier(const Polygon2D<NumType>& p, const float step = 0.05);
   Polygon2D<NumType> multiBezier(const Polygon2D<NumType>& p, const Polygon2D<NumType>& v, const std::vector<float>& k, const float step = 0.05);

   /// \brief Adds an arrow shape to the polygon that goes through the given base points.
   /// \param[in] base_points - Points to place the arrow along.
   /// \param[in] thickness - Thickness at positions of base points.
   /// \param[in] arrow_head - Shape at one side of the arrow.
   /// \param[in] arrow_end - Shape at the other side of the arrow.
   /// \param[in] head_factor - Multiplier for head size.
   /// \param[in] end_factor - Multiplier for end size.
   void createArrow(
      const Polygon2D<NumType>& base_points,
      const std::function<float(const Vector2D<NumType>& point_position, const int point_num)>& thickness = [](const Vector2D<NumType>& point_position, const int point_num) {return 10.0; },
      const Polygon2D<NumType> dock_at_begin = {},
      const Polygon2D<NumType> dock_at_end = {},
      const Polygon2D<NumType>& arrow_head = {},
      const Polygon2D<NumType>& arrow_end = {},
      const Vector2D<NumType>& head_factor = {1, 1},
      const Vector2D<NumType>& end_factor = {1, 1});

   void createArrow(
      const Polygon2D<NumType>& base_points, 
      const float thickness,
      const Polygon2D<NumType> dock_at_begin = {},
      const Polygon2D<NumType> dock_at_end = {},
      const Polygon2D<NumType>& arrow_head = {},
      const Polygon2D<NumType>& arrow_end = {},
      const Vector2D<NumType>& head_factor = {1, 1},
      const Vector2D<NumType>& end_factor = {1, 1});

   /// \brief Calculates for the edge of a polyline intersection points around
   ///  the actual point where an embedding polygon would go to emulate line thickness.
   ///  Returns the two intersection points.
   static Polygon2D<NumType> lineThicknessIntersectionPoints(const Vector2D<NumType>& p1, const Vector2D<NumType>& p2, const Vector2D<NumType>& p3, const float dicke);

   static void createArrowEnd(const Polygon2D<NumType>& base_points, const Polygon2D<NumType>& arrow_end,
                              const Vector2D<NumType>& end_factor, const float thick,
                              Polygon2D<NumType>& pointList, const bool head);

   static std::vector<Polygon2D<NumType>> gestrichelterPfeil(
      const Polygon2D<NumType>& punkte,
      const float strichLenFaktor,
      const std::function<float(const Vector2D<NumType>& point_position, const int point_num)>& thickness = [](const Vector2D<NumType>& point_position, const int point_num) {return 10.0; },
      const Polygon2D<NumType>& pfeilAnfang = ARROW_END_PLAIN_LINE,
      const Polygon2D<NumType>& pfeilEnde = ARROW_END_PLAIN_LINE,
      const Vector2D<NumType>&  anfFaktor = { 1, 1 },
      const Vector2D<NumType>&  endFaktor = { 1, 1 });

   static std::vector<Polygon2D<NumType>> gestrichelterPfeil(
      const Polygon2D<NumType>& punkte,
      const float strichLenFaktor,
      const float thickness,
      const Polygon2D<NumType>& pfeilAnfang = ARROW_END_PLAIN_LINE,
      const Polygon2D<NumType>& pfeilEnde = ARROW_END_PLAIN_LINE,
      const Vector2D<NumType>& anfFaktor = { 1, 1 },
      const Vector2D<NumType>& endFaktor = { 1, 1 });

   static std::vector<Polygon2D<NumType>> erzeugePfeilsegmente(
      const Polygon2D<NumType>& punkte,
      const std::function<float(const Vector2D<NumType>& point_position, const int point_num)>& thickness,
      std::vector<int>& beginn,
      std::vector<int>& ende,
      std::vector<Polygon2D<NumType>>& pfeilAnfang,
      std::vector<Polygon2D<NumType>>& pfeilEnde,
      std::vector<Vector2D<NumType>>& faktorAnfang,
      std::vector<Vector2D<NumType>>& faktorEnde);

private:
   static bool isInside(const Vector2D<NumType>& a, const Vector2D<NumType>& b, const Vector2D<NumType>& c);
   static Vector2D<NumType> intersection(const Vector2D<NumType>& p, const Vector2D<NumType>& q, const Vector2D<NumType>& a, const Vector2D<NumType>& b);

   std::shared_ptr<Rectangle2D<NumType>> bounding_box_ = std::make_shared<Rectangle2D<NumType>>();

   void createBoundingBox() const;
   void extendBoundingBox(const Point& pt) const;
};

typedef Polygon2D<float> Pol2Df;
typedef Polygon2D<double> Pol2Dd;
typedef Polygon2D<int> Pol2Di;
typedef Polygon2D<long> Pol2Dl;
typedef Pol2Df Pol2D;

/// Some predefined arrow ends.
const Pol2D ARROW_END_PLAIN_LINE{};
const Pol2D ARROW_END_PPT_STYLE_1{ { 1.8, -0.7 }, {0, 5}, {-1.8, -0.7} };
const Pol2D ARROW_END_PPT_STYLE_2{ {1.8, 0}, {0, 7}, {-1.8, 0} };
const Pol2D ARROW_END_PPT_STYLE_3{ {2.3, -1.7}, {2.85, -1.7}, {0, 1.1}, {-2.85, -1.7}, {-2.3, -1.7} };
const Pol2D ARROW_END_DOUBLE_SPIKE{ {0, -0.3} };
// const Pol2D ARROW_END_SPHERE = eas.math.geometry.Geometry2D.bezier( // TODO
//         new Vector2D(0.5, 0),
//         new Vector2D(5, 5),
//         new Vector2D(-5, 5),
//         new Vector2D(-0.5, 0),
//         0.0001);

template<class NumType>
inline Polygon2D<NumType>::Polygon2D(const std::vector<Vector2D<NumType>>& p)
   : points_(p.begin(), p.end()) {}

template<class NumType>
inline Polygon2D<NumType>::Polygon2D(const Polygon2D<NumType>& other)
   : Polygon2D<NumType>(other.points_) {}

template<class NumType>
inline Polygon2D<NumType>::Polygon2D(std::initializer_list<Vector2D<NumType>> vecs)
   : points_(vecs) {}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::toPol() const
{
   return Polygon2D<NumType>(*this);
}

template<class NumType>
inline std::string Polygon2D<NumType>::serialize() const
{
   std::string s = "(";

   if (!points_.empty()) {
      s += points_[0].serialize();
   }

   for (int i = 1; i < points_.size(); i++) {
      s += ", " + points_[i].serialize();
   }

   s += ")";

   return s;
}

template<class NumType>
inline void Polygon2D<NumType>::add(const Vector2D<NumType>& p)
{
   points_.push_back(p);
   extendBoundingBox(p); // This will create the bb if non-existing. May be inefficient for large Polygons without need for bb.
}

template<class NumType>
inline void Polygon2D<NumType>::addAll(const Polygon2D<NumType>& p)
{
   for (const auto& point : p.points_) {
      add(point);
   }
}

template<class NumType>
inline void Polygon2D<NumType>::add(const int position, const Vector2D<NumType>& p)
{
   points_.insert(points_.begin() + position, p);
   extendBoundingBox(p); // This will create the bb if non-existing. May be inefficient for large Polygons without need for bb.
}

template<class NumType>
inline std::shared_ptr<Rectangle2D<NumType>> Polygon2D<NumType>::getBoundingBox() const
{
   if (bounding_box_->isInvalid()) {
      createBoundingBox();
   }

   return bounding_box_;
}

template<class NumType>
inline  std::vector<Vector2D<NumType>> Polygon2D<NumType>::intersect(const LineSegment2D<NumType>& s1) const
{
   std::vector<Vector2D<NumType>> vec;
   LineSegment2D<NumType> s2({ points_[points_.size() - 1], points_[0] });
   bool flip = false;

   for (int i = 1; i <= points_.size(); i++) {
      auto schnitt = s1.intersect(s2);

      if (schnitt) {
         vec.push_back(*schnitt);
      }

      if (i >= points_.size()) break;

      if (flip) {
         s2.end_ = points_[i];
      }
      else {
         s2.begin_ = points_[i];
      }

      flip = !flip;
   }

   return vec;
}

template<class NumType>
inline std::vector<Vector2D<NumType>> Polygon2D<NumType>::intersect(const Polygon2D<NumType>& other) const
{
   std::vector<Vector2D<NumType>> vec;
   LineSegment2D<NumType> s2({ points_[points_.size() - 1], points_[0] });
   bool flip = false;

   for (int i = 1; i < points_.size(); i++) {
      auto schnitt = other.intersect(s2);

      vec.insert(vec.end(), schnitt.begin(), schnitt.end());

      if (flip) {
         s2.end_ = points_[i];
      }
      else {
         s2.begin_ = points_[i];
      }

      flip = !flip;
   }

   return vec;
}

template<class NumType>
inline bool Polygon2D<NumType>::hasCommonPointWith(const LineSegment2D<NumType>& s1) const
{
   return isPointWithin(s1.begin_) || isPointWithin(s1.end_);
}

template<class NumType>
inline bool Polygon2D<NumType>::hasCommonPointWith(const Polygon2D<NumType>& other) const
{
   const Polygon2D<NumType>& p1 = points_.size() < other.points_.size() ? other : *this;
   const Polygon2D<NumType>& p2 = points_.size() < other.points_.size() ? *this : other;

   for (const auto& p : p1.points_) {
      if (p2.isPointWithin(p)) {
         return true;
      }
   }

   return false;
}

template<class NumType>
inline void Polygon2D<NumType>::translate(const Vector2D<NumType>& vek)
{
   for (Point& pVek : points_) {
      pVek.translate(vek);
   }

   if (!bounding_box_->isInvalid()) {
      bounding_box_->translate(vek);
   }
}

template<class NumType>
inline void Polygon2D<NumType>::scale(const NumType scal)
{
   scale(scal, getBoundingBox()->getCenter());
}

template<class NumType>
inline void Polygon2D<NumType>::scale(const NumType scal, const Vector2D<NumType>& center)
{
   scale({ scal, scal }, center);
}

template<class NumType>
inline void Polygon2D<NumType>::scale(const Vector2D<NumType>& scal)
{
   scale(scal, getBoundingBox()->getCenter());
}

template<class NumType>
inline void Polygon2D<NumType>::scale(const Vector2D<NumType>& scal, const Vector2D<NumType>& center)
{
   for (Point& p : points_) {
      p.scale(scal, Vec2Df::ScalingOption::none, center);
   }

   bounding_box_->makeInvalid(); // TODO: Scale bounding_box_, too.
}

template<class NumType>
inline void Polygon2D<NumType>::rotate(const float angle_rad)
{
   rotate(angle_rad, getBoundingBox()->getCenter());
}

template<class NumType>
inline void Polygon2D<NumType>::rotate(const float angle_rad, const Vector2D<NumType>& center)
{
   for (Vector2D<NumType>& p : points_) {
      p.rotate(angle_rad, center);
   }

   bounding_box_->makeInvalid();
}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::normalize(const bool close) const
{
   if (points_.size() <= 1) {
      return *this;
   }

   auto pts = points_;

   if (close) {
      pts.push_back(pts[0]);
   }

   Polygon2D<NumType> normArr;
   std::vector<NumType> distances;
   distances.reserve(pts.size());
   Vector2D<NumType> latestV = pts[0];
   NumType currentDist;
   NumType latestDist = 0;
   Vector2D<NumType> currentDir;
   NumType avgDistance;
   int i = 0;

   for (const Vector2D<NumType>& currentV : pts) {
      currentDist = currentV.distance(latestV) + latestDist;
      distances.push_back(currentDist);
      latestDist = currentDist;
      latestV = currentV;
   }

   avgDistance = distances[distances.size() - 1] / (distances.size() - 1);

   Vector2D<NumType> currentV(pts[0]);
   normArr.add(currentV);
   currentDist = 0;

   while (i < pts.size()) {
      if (i + 1 < pts.size()) {
         currentDir = pts[i + 1];
         currentDir.sub(pts[i]);
         currentDir.setLength(avgDistance);

         while (currentDist < distances[i + 1]) {
            currentV = Point(currentV);
            currentV.translate(currentDir);
            normArr.add(currentV);
            currentDist += avgDistance;
         }
      }

      i++;
   }

   assert((normArr.points_.size() - pts.size() == 1 || normArr.points_.size() - pts.size() == 0));

   return normArr;
}

template<class NumType>
inline bool Polygon2D<NumType>::isPointWithin(const Vector2D<NumType>& punkt) const
{
   if (points_.empty()) {
      return false;
   }

   bool inside = false;

   NumType x1 = points_[points_.size() - 1].x;
   NumType y1 = points_[points_.size() - 1].y;
   NumType x2 = points_[0].x;
   NumType y2 = points_[0].y;
   bool startUeber;
   bool endeUeber;
   
   if (y1 >= punkt.y) {
      startUeber = true;
   }
   else {
      startUeber = false;
   }
   
   for (int i = 1; i <= points_.size(); i++) {
      if (y2 >= punkt.y) {
         endeUeber = true;
      }
      else {
         endeUeber = false;
      }
   
      if (startUeber != endeUeber) {
         if ((y2 - punkt.y) * (x2 - x1) <= (y2 - y1) * (x2 - punkt.x)) {
            if (endeUeber) {
               inside = !inside;
            }
         }
         else {
            if (!endeUeber) {
               inside = !inside;
            }
         }
      }
   
      if (i < points_.size()) {
         startUeber = endeUeber;
         y1 = y2;
         x1 = x2;
         x2 = points_[i].x;
         y2 = points_[i].y;
      }
   }
   
   return inside;
}

template<class NumType>
inline Vector2D<NumType> Polygon2D<NumType>::centerPoint()
{
   return { getBoundingBox()->upper_left_.x + getBoundingBox()->getWidth() / 2,
            getBoundingBox()->upper_left_.y + getBoundingBox()->getHeight() / 2 };
}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::getCounterClockwiseOrderedIfStarShaped()
{
   Vector2D<NumType> richt0(points_[0]);
   Vector2D<NumType> richt1(points_[1]);

   richt0.sub(centerPoint());
   richt1.sub(centerPoint());

   if (richt0.orientation(richt1)) {
      return *this;
   }
   else {
      return getReverted();
   }
}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::getReverted() const
{
   Polygon2D<NumType> p;

   for (int i = points_.size() - 1; i >= 0; i--) {
      p.add(points_[i]);
   }
   
   return p;
}

template<class NumType>
inline void Polygon2D<NumType>::translateRootPointToMiddle()
{
   Vector2D<NumType> v(centerPoint());
   v.mult(-1);
   translate(v);
}

template<class NumType>
inline void Polygon2D<NumType>::insertNodesOnExistingEdge(const int edg1Num, const float nodesToInsert)
{
   int edg2Num = (edg1Num + 1) % points_.size();

   Vector2D<NumType> dir(points_[edg2Num]);
   dir.sub(points_[edg1Num]);
   float length = points_[edg2Num].distance(points_[edg1Num]);
   dir.setLength(length / (nodesToInsert + 1));
   Vector2D<NumType> vec(points_[edg1Num]);

   for (int i = 0; i < nodesToInsert; i++) {
      vec.translate(dir);
      add(edg1Num + i + 1, Vector2D<NumType>(vec));
   }
}

template<class NumType>
inline void Polygon2D<NumType>::insertNodesOnExistingEdges(const int nodesToInsert, const bool close_polygon)
{
   for (int i = points_.size() - (1 + !close_polygon); i >= 0; i--) {
      insertNodesOnExistingEdge(i, nodesToInsert);
   }
}

template<class NumType>
inline bool Polygon2D<NumType>::isConvex() const
{
   int size = points_.size();

   for (int i = 0; i < size; i++) {
      Vector2D<NumType> x = points_[i];
      Vector2D<NumType> y = points_[(i + 1) % size];
      Vector2D<NumType> z = points_[(i + 2) % size];

      // 3d cross product points up or down? (TODO: Externalize to Vector3D class, once ported.)
      if ((z.x - x.x) * (y.y - x.y) - (y.x - x.x) * (z.y - x.y) >= 0) {
         return false;
      }
   }

   return true;
}

template<class NumType>
inline bool Polygon2D<NumType>::isInside(const Vector2D<NumType>& a, const Vector2D<NumType>& b, const Vector2D<NumType>& c)
{
   return (a.x - c.x) * (b.y - c.y) > (a.y - c.y) * (b.x - c.x);
}

template<class NumType>
inline Vector2D<NumType> Polygon2D<NumType>::intersection(const Vector2D<NumType>& p, const Vector2D<NumType>& q, const Vector2D<NumType>& a, const Vector2D<NumType>& b)
{
   NumType A1 = q.y - p.y;
   NumType B1 = p.x - q.x;
   NumType C1 = A1 * p.x + B1 * p.y;

   NumType A2 = b.y - a.y;
   NumType B2 = a.x - b.x;
   NumType C2 = A2 * a.x + B2 * a.y;

   NumType det = A1 * B2 - A2 * B1;
   NumType x = (B2 * C1 - B1 * C2) / det;
   NumType y = (A1 * C2 - A2 * C1) / det;

   return { x, y };
}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::clipPolygonsOneConvex(const Polygon2D<NumType>& p2) const
{
   if (!p2.isConvex()) {
      if (isConvex()) {
         return p2.clipPolygonsOneConvex(*this);
      }
      else {
         return {}; // Error case. TODO: Make class "failable" and throw error.
      }
   }

   Polygon2D<NumType> result(*this);

   int len = p2.points_.size();
   for (int i = 0; i < len; i++) {
      int len2 = result.points_.size();
      Polygon2D<NumType> input(result);
      result = Polygon2D<NumType>();

      Vector2D<NumType> A(p2.points_[(i + len - 1) % len]);
      Vector2D<NumType> B(p2.points_[i]);

      for (int j = 0; j < len2; j++) {
         Vector2D<NumType> P(input.points_[(j + len2 - 1) % len2]);
         Vector2D<NumType> Q(input.points_[j]);

         if (isInside(A, B, Q)) {
            if (!isInside(A, B, P)) {
               result.add(intersection(P, Q, A, B));
            }

            result.add(Q);
         }
         else if (isInside(A, B, P))
            result.add(intersection(P, Q, A, B));
      }
   }

   return result;
}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::bezier(const Vector2D<NumType>& p0, const Vector2D<NumType>& p1, const Vector2D<NumType>& p2, const Vector2D<NumType>& p3, const float step)
{
   Vector2D<NumType> punkt;
   Vector2D<NumType> dreiP0(p0);
   Vector2D<NumType> dreiP1(p1);
   Vector2D<NumType> dreiP2(p2);
   Vector2D<NumType> sechsP1(p1);
   dreiP0.mult(3);
   dreiP1.mult(3);
   dreiP2.mult(3);
   sechsP1.mult(6);
   Vector2D<NumType> punkt2, punkt3;
   NumType tQuad, tKub;

   for (double t = 0; t <= 1; t += step) {
      tQuad = t * t;
      tKub = t * tQuad;
           
      punkt = Vector2D<NumType>(dreiP1);
      punkt.sub(p0);
      punkt.sub(dreiP2);
      punkt.translate(p3);
      punkt.mult(tKub);
           
      punkt2 = Vector2D<NumType>(dreiP0);
      punkt2.sub(sechsP1);
      punkt2.translate(dreiP2);
      punkt2.mult(tQuad);
           
      punkt3 = Vector2D<NumType>(dreiP1);
      punkt3.sub(dreiP0);
      punkt3.mult(t);
           
      punkt.translate(punkt2);
      punkt.translate(punkt3);
      punkt.translate(p0);
           
      add(punkt);
   }

   return *this;
}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::multiBezier(const Polygon2D<NumType>& p, const float step)
{
   Polygon2D<NumType> directions{};
   std::vector<float> k{};

   for (size_t i = 0; i < p.points_.size() - 1; ++i) {
      Vector2D<NumType> dir = p.points_[i + 1];
      dir.sub(p.points_[i]);
      //dir.normalize();
      directions.add(dir);

      if (i) k.push_back(1);
   }

   if (!directions.points_.empty()) {
      directions.add(directions.points_.back());
   }

   return multiBezier(p, directions, k, step);
}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::multiBezier(const Polygon2D<NumType>& p, const Polygon2D<NumType>& v, const std::vector<float>& k, const float step)
{
   assert((p.points_.size() >= 2 && p.points_.size() == v.points_.size() && v.points_.size() == k.size() + 2));

   Polygon2D<NumType> bez;
   Vector2D<NumType> v1, v2;
        
   v1 = Vector2D<NumType>(v.points_[0]);
   v2 = Vector2D<NumType>(p.points_[1]);
   v1.translate(p.points_[0]);
   v2.sub(v.points_[1]);
        
   bez.bezier(p.points_[0], v1, v2, p.points_[1], step);
   points_.insert(points_.end(), bez.points_.begin(), bez.points_.end());
        
   for (int i = 1; i < p.points_.size() - 1; i++) {
      v1 = Vector2D<NumType>(v.points_[i]);
      v2 = Vector2D<NumType>(p.points_[i + 1]);
      v1.mult(k[i - 1]);
      v1.translate(p.points_[i]);
      v2.sub(v.points_[i + 1]);
      bez = Polygon2D<NumType>();
      bez.bezier(p.points_[i], v1, v2, p.points_[i + 1], step);
      points_.insert(points_.end(), bez.points_.begin(), bez.points_.end());
   }

   return *this;
}


template<class NumType>
inline void Polygon2D<NumType>::createArrow(
   const Polygon2D<NumType>& base_points, 
   const std::function<float(const Vector2D<NumType>& point_position, const int point_num)>& thickness, 
   const Polygon2D<NumType> dock_at_begin,
   const Polygon2D<NumType> dock_at_end,
   const Polygon2D<NumType>& arrow_head,
   const Polygon2D<NumType>& arrow_end, 
   const Vector2D<NumType>& head_factor, 
   const Vector2D<NumType>& end_factor)
{
   assert(base_points.points_.size() >= 2);

   Polygon2D<NumType> pointList2;
   float thickness_first = thickness(base_points.points_[0], 0);

   // Arrow head.
   addAll(dock_at_begin);
   createArrowEnd(base_points, arrow_head, head_factor, thickness_first, pointList2, true);

   Vector2D<NumType> c1(base_points.points_[1]);
   c1.sub(base_points.points_[0]);
   c1.ortho();
   c1.normalize();
   c1.mult(thickness_first / 2);

   Vector2D<NumType> p01(base_points.points_[0]);
   Vector2D<NumType> p02(p01);

   p01.translate(c1);
   p02.sub(c1);

   add(p01);
   pointList2.add(p02);

   for (int i = 1; i < base_points.points_.size() - 1; i++) {
      Polygon2D<NumType> tempPoints = lineThicknessIntersectionPoints(base_points.points_[i - 1], base_points.points_[i], base_points.points_[i + 1], thickness(base_points.points_[i], i));
      LineSegment2D<NumType> p(tempPoints.points_[0], tempPoints.points_[1]);
      LineSegment2D<NumType> q(points_[points_.size() - 1], pointList2.points_[pointList2.points_.size() - 1]);

      add(tempPoints.points_[0]);
      pointList2.add(tempPoints.points_[1]);
      
      if (p.intersect(q)) {
         Failable::getSingleton()->addDebug("Segment could not be placed without an intersection at point: " + base_points.points_[i].serialize());
      }
   }

   Vector2D<NumType> c2(base_points.points_[base_points.points_.size() - 1]);
   float thickness_last = thickness(base_points.points_[base_points.points_.size() - 1], base_points.points_.size() - 1);
   c2.sub(base_points.points_[base_points.points_.size() - 2]);
   c2.ortho();
   c2.normalize();
   c2.mult(thickness_last / 2);

   Vector2D<NumType> p11(base_points.points_[base_points.points_.size() - 1]);
   Vector2D<NumType> p12(p11);

   p11.translate(c2);
   p12.sub(c2);

   add(p11);
   pointList2.add(p12);

   // Arrow end.
   addAll(dock_at_end);
   createArrowEnd(base_points, arrow_end, end_factor, thickness_last, *this, false);

   for (int i = pointList2.points_.size() - 1; i >= 0; i--) {
      add(pointList2.points_[i]);
   }
}

template<class NumType>
inline void Polygon2D<NumType>::createArrow(
   const Polygon2D<NumType>& base_points, 
   const float thickness,
   const Polygon2D<NumType> dock_at_begin,
   const Polygon2D<NumType> dock_at_end,
   const Polygon2D<NumType>& arrow_head,
   const Polygon2D<NumType>& arrow_end, 
   const Vector2D<NumType>& head_factor, 
   const Vector2D<NumType>& end_factor)
{
   createArrow(base_points,
      [thickness](const Vector2D<NumType>& point_position, const int point_num){return thickness;},
      dock_at_begin,
      dock_at_end,
      arrow_head,
      arrow_end,
      head_factor,
      end_factor);
}

template<class NumType>
inline Polygon2D<NumType> Polygon2D<NumType>::lineThicknessIntersectionPoints(const Vector2D<NumType>& p1, const Vector2D<NumType>& p2, const Vector2D<NumType>& p3, const float dicke)
{
   Polygon2D<NumType> pkte;
         
   Vector2D<NumType> v1(p2.x - p1.x, p2.y - p1.y);
   Vector2D<NumType> v2(p3.x - p2.x, p3.y - p2.y);
   Vector2D<NumType> c1(v1.y, -v1.x);
   Vector2D<NumType> c2(v2.y, -v2.x);
   Vector2D<NumType> p11;
   Vector2D<NumType> p12; 
   Vector2D<NumType> p01;
   Vector2D<NumType> p02;
         
   c1.mult(dicke / (c1.length() * 2));
   c2.mult(dicke / (c2.length() * 2));
         
   p11 = Vector2D<NumType>(p1);
   p12 = Vector2D<NumType>(p3);
   p01 = Vector2D<NumType>(p1);
   p02 = Vector2D<NumType>(p3);
         
   p11.translate(c1);
   p12.translate(c2);
   p01.sub(c1);
   p02.sub(c2);
         
   v1.setLength(p1.distance(p2));
   v2.setLength(p2.distance(p3));
         
   Line2D<NumType> g1(p11, v1);
   Line2D<NumType> h1(p12, v2);
   Line2D<NumType> g2(p01, v1);
   Line2D<NumType> h2(p02, v2);
         
   pkte.add(*g1.intersectSpecial(h1));
   pkte.add(*g2.intersectSpecial(h2));
         
   return pkte;
}

template<class NumType>
inline void Polygon2D<NumType>::createArrowEnd(
   const Polygon2D<NumType>& base_points, 
   const Polygon2D<NumType>& arrow_end, 
   const Vector2D<NumType>& end_factor, 
   const float thick,
   Polygon2D<NumType>& pointList, 
   const bool head)
{
   Vector2D<NumType> normEnd(base_points.points_[head ? 0 : base_points.points_.size() - 1]);
   normEnd.sub(base_points.points_[head ? 1 : base_points.points_.size() - 2]);
   normEnd.normalize();
   Vector2D<NumType> orthoEnd(normEnd);
   orthoEnd.ortho();
   int iter = head ? -1 : 1;

   for (int i = head ? arrow_end.points_.size() - 1 : 0; head ? i >= 0 : i < arrow_end.points_.size(); i += iter) {
      Vector2D<NumType> currentVec(arrow_end.points_[i]);
      Vector2D<NumType> currentVecX(orthoEnd);
      Vector2D<NumType> currentVecY(normEnd);
      currentVecX.mult(currentVec.x * end_factor.x * thick);
      currentVecY.mult(currentVec.y * end_factor.y * thick);
      currentVec = Vector2D<NumType>(currentVecX);
      currentVec.translate(currentVecY);
      currentVec.translate(base_points.points_[head ? 0 : base_points.points_.size() - 1]);
      pointList.add(currentVec);
   }
}

template<class NumType>
inline void Polygon2D<NumType>::createBoundingBox() const
{
   if (points_.empty()) {
      bounding_box_->makeInvalid();
   }
   else {
      bounding_box_->upper_left_ = points_[0];
      bounding_box_->lower_right_ = points_[0];
   }

   for (int i = 1; i < points_.size(); i++) {
      extendBoundingBox(points_[i]);
   }
}

template<class NumType>
inline void Polygon2D<NumType>::extendBoundingBox(const Point& pt) const
{
   if (pt.x != pt.x || pt.y != pt.y) {
      Failable::getSingleton()->addError("NAN occurred in polygon generation.");
      return;
   }

   if (bounding_box_->isInvalid()) {
      createBoundingBox();
   }

   if (pt.x < bounding_box_->upper_left_.x) {
      bounding_box_->upper_left_.x = pt.x;
   }
   else if (pt.x > bounding_box_->lower_right_.x) {
      bounding_box_->lower_right_.x = pt.x;
   }

   if (pt.y < bounding_box_->upper_left_.y) {
      bounding_box_->upper_left_.y = pt.y;
   }
   else if (pt.y > bounding_box_->lower_right_.y) {
      bounding_box_->lower_right_.y = pt.y;
   }
}

template<class NumType>
inline std::vector<Polygon2D<NumType>> Polygon2D<NumType>::gestrichelterPfeil(
   const Polygon2D<NumType>& punkte, 
   const float strichLenFaktor, 
   const float thickness, 
   const Polygon2D<NumType>& pfeilAnfang, 
   const Polygon2D<NumType>& pfeilEnde, 
   const Vector2D<NumType>& anfFaktor, 
   const Vector2D<NumType>& endFaktor)
{
   return gestrichelterPfeil(
      punkte, 
      strichLenFaktor, 
      [thickness](const Vector2D<NumType>& point_position, const int point_num) { return thickness; },
      pfeilAnfang,
      pfeilEnde,
      anfFaktor,
      endFaktor);
}

/**
 * Erzeugt einen gestrichelten Pfeil mit verschiedenen Segmenttypen.
 *
 * @param punkte           Die Punkte des Pfeils.
 * @param dicken           Die Dicken des Pfeils.
 * @param pfeilAnfang      Das Pfeilanfangspolygon.
 * @param pfeilEnde        Das Pfeilendepolygon.
 * @param anfFaktor        Der Anfangsfaktor.
 * @param endFaktor        Der Endfaktor.
 * @param farben           Eine beliebige Liste sich abwechselnder
 *                         Ausgabemerkmale. Ist hier ein Eintrag
 *                         <code>null</code>, wird kein Ausgabemerkmal
 *                         für die Segmente definiert.
 * @param strichLenFaktor  Multiplikativer Faktor für die Länge eines
 *                         Segments bezogen auf die durchschnittliche
 *                         Dicke des Pfeils.
 * @param params           Die Parameter.
 * @return  Der gestrichelte Pfeil als Menge von Segmentpolygonen.
 */
template<class NumType>
inline std::vector<Polygon2D<NumType>> Polygon2D<NumType>::gestrichelterPfeil(
   const Polygon2D<NumType>& punkte,
   const float strichLenFaktor,
   const std::function<float(const Vector2D<NumType>& point_position, const int point_num)>& thickness,
   const Polygon2D<NumType>& pfeilAnfang,
   const Polygon2D<NumType>& pfeilEnde,
   const Vector2D<NumType>&  anfFaktor,
   const Vector2D<NumType>&  endFaktor) 
{
   const double konstFakt = 3 * strichLenFaktor;
   float strLaeng;
   float durchDick = 0;
   float durchAbst = 0;
   float anzPunkte;
   int j;
   Polygon2D<NumType> anfang;
   Polygon2D<NumType> ende;
   float mehrPunkte;
   float anzSegmente;

   for (int i = 0; i < punkte.points_.size(); i++) {
      durchDick += std::abs(thickness({ 0, 0 }, i));
   }
   durchDick /= punkte.points_.size();

   for (int i = 0; i < punkte.points_.size() - 2; i++) {
      durchAbst += punkte.points_.at(i).distance(punkte.points_.at(i + 1));
   }
   durchAbst /= punkte.points_.size() - 1;
   strLaeng = durchDick * konstFakt;
   anzPunkte = strLaeng / durchAbst;

   if (anzPunkte == 0) {
      return gestrichelterPfeil(
         punkte,
         strichLenFaktor + 1,
         thickness,
         pfeilAnfang,
         pfeilEnde,
         anfFaktor,
         endFaktor);
   }

   std::vector<int> beginns;
   std::vector<int> endes;
   std::vector<Polygon2D<NumType>> pfeilAnfangs;
   std::vector<Polygon2D<NumType>> pfeilEndes;
   std::vector<Vector2D<NumType>> faktorAnfangs;
   std::vector<Vector2D<NumType>> faktorEndes;

   anzSegmente = punkte.points_.size() / anzPunkte;
   mehrPunkte = fmodf(punkte.points_.size(), anzPunkte);
   anzPunkte += mehrPunkte / ((int)anzSegmente + 1);

   for (float i = 0; i <= punkte.points_.size() - anzPunkte; i += anzPunkte) {
      anfang = ARROW_END_PLAIN_LINE;
      ende = ARROW_END_PLAIN_LINE;
      if (i == 0) {
         anfang = pfeilAnfang;
      }
      if (i + anzPunkte > punkte.points_.size() - anzPunkte) {
         ende = pfeilEnde;
         anzPunkte = punkte.points_.size() - i - 1;
      }

      beginns.push_back((int) i);
      endes.push_back((int)(i + anzPunkte));
      pfeilAnfangs.push_back(anfang);
      pfeilEndes.push_back(ende);
      faktorAnfangs.push_back(anfFaktor);
      faktorEndes.push_back(endFaktor);
   }

   return erzeugePfeilsegmente(punkte, thickness, beginns, endes, pfeilAnfangs, pfeilEndes, faktorAnfangs, faktorEndes);
}

/**
 * Gibt eine beliebige Menge von beliebig zusammenhängenden Teilsegmenten
 * eines Pfeils zurück, wobei die Ausgabemerkmale für jedes zus.
 * Teilsegment einzeln eingestellt werden können.
 *
 * @param punkte       Die Punkte, durch die der Pfeil verlaufen soll.
 * @param dicken       Die Dicken des Pfeils an den Stellen der
 *                     Verlaufspunkte.
 * @param segBeschr    Die Beschreibungen einzelner Segmente.
 * @param params       Die Parameter.
 *
 * @return  Die Teilsegmente eines Pfeil aus mehreren Segmenten.
 */
template<class NumType>
inline std::vector<Polygon2D<NumType>> Polygon2D<NumType>::erzeugePfeilsegmente(
   const Polygon2D<NumType>& punkte,
   const std::function<float(const Vector2D<NumType>& point_position, const int point_num)>& thickness,
   std::vector<int>& beginn,
   std::vector<int>& ende,
   std::vector<Polygon2D<NumType>>& pfeilAnfang,
   std::vector<Polygon2D<NumType>>& pfeilEnde,
   std::vector<Vector2D<NumType>>& faktorAnfang,
   std::vector<Vector2D<NumType>>& faktorEnde)
{
   std::vector<Polygon2D<NumType>> segmente;
   Polygon2D<NumType> zeichnen;

   for (int i = 0; i < beginn.size(); i++) {
      Polygon2D<NumType> aktPol;
      Polygon2D<NumType> aktPunkte;

      for (int j = beginn[i]; j <= ende[i]; j++) {
         aktPunkte.add(punkte.points_.at(j));
      }

      aktPol.createArrow(
         aktPunkte,
         thickness,
         {},
         {},
         pfeilAnfang[i],
         pfeilEnde[i],
         faktorAnfang[i],
         faktorEnde[i]);

      segmente.push_back(aktPol);
   }

   return segmente;
}

} // vfm
