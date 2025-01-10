//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include <initializer_list>
#include "abstract_object.h"
#include <cmath>
#include <memory>


namespace vfm {

template<class NumType>
class Line2D;

template<class NumType>
class LineSegment2D : public Object2D<NumType> {
public:
   using Point = typename Object2D<NumType>::Point;
   using PointVec = typename Object2D<NumType>::PointVec;

   Point begin_;
   Point end_;

   LineSegment2D(const Vector2D<NumType>& b, const Vector2D<NumType>& e);
   LineSegment2D(const LineSegment2D<NumType>& other);
   LineSegment2D(std::initializer_list<Vector2D<NumType>>& vecs);

   Polygon2D<NumType> toPol() const override;
   std::string serialize() const override;
   Vector2D<NumType> getNormDir() const;
   float length() const;
   Vector2D<NumType> intersect(const Polygon2D<NumType>& p);
   std::shared_ptr<Vector2D<NumType>> intersect(const LineSegment2D<NumType>& q) const;
   bool isLeftOf(const Vector2D<NumType>& point) const;

private:
   bool isPointOnProlongedLineAlsoOnSegment(const Vector2D<NumType> point) const;
};

typedef LineSegment2D<float> LinSeg2Df;
typedef LineSegment2D<double> LinSeg2Dd;
typedef LineSegment2D<int> LinSeg2Di;
typedef LineSegment2D<long> LinSeg2Dl;
typedef LinSeg2Df LinSeg2D;

template<class NumType>
inline LineSegment2D<NumType>::LineSegment2D(const Vector2D<NumType>& b, const Vector2D<NumType>& e)
   : begin_(b), end_(e) {}

template<class NumType>
inline LineSegment2D<NumType>::LineSegment2D(const LineSegment2D<NumType>& other)
   : LineSegment2D<NumType>(other.begin_, other.end_) {}

template<class NumType>
inline LineSegment2D<NumType>::LineSegment2D(std::initializer_list<Vector2D<NumType>>& vecs) 
   : LineSegment2D<NumType>(vecs.begin(), vecs.begin() + 1) {}

template<class NumType>
inline Polygon2D<NumType> LineSegment2D<NumType>::toPol() const
{
   return Polygon2D<NumType>({ begin_, end_ });
}

template<class NumType>
inline std::string LineSegment2D<NumType>::serialize() const
{
   return begin_.serialize() + " --- " + end_.serialize();
}

template<class NumType>
inline Vector2D<NumType> LineSegment2D<NumType>::getNormDir() const
{
   Vector2D<NumType> dir = end_;
   dir.sub(begin_);
   dir.normalize();
   return dir;
}

template<class NumType>
inline float LineSegment2D<NumType>::length() const
{
   return begin_.distance(end_);
}

template<class NumType>
inline Vector2D<NumType> LineSegment2D<NumType>::intersect(const Polygon2D<NumType>& p) 
{
   return p.intersect(*this);
}

template<class NumType>
inline std::shared_ptr<Vector2D<NumType>> LineSegment2D<NumType>::intersect(const LineSegment2D<NumType>& q) const
{
   Vector2D<NumType> vp = getNormDir(), vq = q.getNormDir();
   Line2D<NumType> g{ begin_, vp }, h{ q.begin_, vq };

   std::shared_ptr<Vector2D<NumType>> inter = g.intersect(h);

   if (!inter) {
      return nullptr;
   }

   if (isPointOnProlongedLineAlsoOnSegment(*inter) && q.isPointOnProlongedLineAlsoOnSegment(*inter)) {
      return inter;
   }
   else {
      return nullptr;
   }
}

template<class NumType>
inline bool LineSegment2D<NumType>::isLeftOf(const Vector2D<NumType>& point) const
{
   NumType x2 = begin_.x - end_.x;
   NumType x3 = begin_.x - point.x;
   NumType y2 = begin_.y - end_.y;
   NumType y3 = begin_.y - point.y;

   return (x2 * y3 - y2 * x3 < 0);
}

template<class NumType>
inline bool LineSegment2D<NumType>::isPointOnProlongedLineAlsoOnSegment(const Vector2D<NumType> point) const
{
   double aq = begin_.distance(point);
   double ab = begin_.distance(end_);
   double qb = point.distance(end_);

   return aq <= ab && qb <= ab;
}

} // vfm
