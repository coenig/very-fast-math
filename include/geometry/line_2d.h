//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "line_segment_2d.h"
#include "vector_2d.h"
#include "abstract_object.h"
#include <initializer_list>
#include <cmath>
#include <memory>


namespace vfm {

template<class NumType>
class Line2D : public Object2D<NumType> {
public:
   using Point = typename Object2D<NumType>::Point;
   using PointVec = typename Object2D<NumType>::PointVec;

   Point base_point_;
   Point direction_;

   Line2D(const Vector2D<NumType>& base_point, const Vector2D<NumType>& direction);
   Line2D(const Line2D<NumType>& other);
   explicit Line2D(const LineSegment2D<NumType>& other);
   Line2D(std::initializer_list<Vector2D<NumType>>& vecs);

   Polygon2D<NumType> toPol() const override;
   std::string serialize() const override;
   std::shared_ptr<Vector2D<NumType>> intersectSpecial(const Line2D<NumType>& h) const;
   std::shared_ptr<Vector2D<NumType>> intersect(const Line2D<NumType>& h) const;
};

typedef Line2D<float> Lin2Df;
typedef Line2D<double> Lin2Dd;
typedef Line2D<int> Lin2Di;
typedef Line2D<long> Lin2Dl;
typedef Lin2Df Lin2D;

template<class NumType>
inline Line2D<NumType>::Line2D(const Vector2D<NumType>& base_point, const Vector2D<NumType>& direction) 
   : base_point_(base_point), direction_(direction) {}

template<class NumType>
inline Line2D<NumType>::Line2D(const Line2D<NumType>& other) 
   : Line2D<NumType>(other.base_point_, other.direction_) {}

template<class NumType>
inline Line2D<NumType>::Line2D(const LineSegment2D<NumType>& other)
   : Line2D<NumType>(other.begin_, other.end_)
{ 
   direction_ = other.end_;
   direction_.sub(other.begin_); 
}

template<class NumType>
inline Line2D<NumType>::Line2D(std::initializer_list<Vector2D<NumType>>& vecs) 
   : Line2D<NumType>(vecs.begin(), vecs.begin() + 1) {}

template<class NumType>
inline Polygon2D<NumType> Line2D<NumType>::toPol() const
{
   Point p1(base_point_), p2(base_point_);
   p2.add(direction_);
   return Polygon2D<NumType>({ p1, p2 });
}

template<class NumType>
inline std::string Line2D<NumType>::serialize() const
{
   return base_point_.serialize() + " ---> " + direction_.serialize();
}

template<class NumType>
inline std::shared_ptr<Vector2D<NumType>> Line2D<NumType>::intersectSpecial(const Line2D<NumType>& h) const
{
   auto ptr = intersect(h);

   if (ptr) {
      return ptr;
   }
   else {
      auto vec = std::make_shared<Vector2D<NumType>>((base_point_.x + h.base_point_.x) / 2, (base_point_.y + h.base_point_.y) / 2);
      return vec;
   }
}

template<class NumType>
inline std::shared_ptr<Vector2D<NumType>> Line2D<NumType>::intersect(const Line2D<NumType>& h) const
{
   NumType t;
   NumType div;
   const NumType nullKonst = 0.0001;
   std::shared_ptr<Vector2D<NumType>> q = nullptr;
   const Vector2D<NumType> p1 = base_point_;
   const Vector2D<NumType> v1 = direction_;
   const Vector2D<NumType> p2 = h.base_point_;
   const Vector2D<NumType> v2 = h.direction_;
   t = v1.y * (p2.x - p1.x) - v1.x * (p2.y - p1.y);
   div = v1.x * v2.y - v2.x * v1.y;

   if (std::abs(div) > nullKonst) {
      t /= div;
      q = std::make_shared<Vector2D<NumType>>(v2);
      q->mult(t);
      q->translate(p2);
   }

   return q;
}

} // vfm
