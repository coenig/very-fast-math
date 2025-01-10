//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "vector_3d.h"
#include "line_segment_3d.h"
#include "abstract_object.h"
#include <initializer_list>
#include <cmath>
#include <memory>


namespace vfm {

template<class NumType>
class Line3D : public GeometricObject<NumType> {
public:
   using Point = Vector3D<NumType>;
   using PointVec = std::vector<Point>;

   Point base_point_;
   Point direction_;

   Line3D(const Point& base_point, const Point& direction);
   Line3D(const Line3D<NumType>& other);
   explicit Line3D(const LineSegment3D<NumType>& other);
   Line3D(std::initializer_list<Point>& vecs);

   std::string serialize() const override;
   std::shared_ptr<Vector2D<NumType>> intersect(const Line3D<NumType>& h) const;
};

typedef Line3D<float> Lin3Df;
typedef Line3D<double> Lin3Dd;
typedef Line3D<int> Lin3Di;
typedef Line3D<long> Lin3Dl;
typedef Lin3Df Lin3D;

template<class NumType>
inline Line3D<NumType>::Line3D(const Point& base_point, const Point& direction)
   : base_point_(base_point), direction_(direction) {}

template<class NumType>
inline Line3D<NumType>::Line3D(const Line3D<NumType>& other) 
   : Line3D<NumType>(other.base_point_, other.direction_) {}

template<class NumType>
inline Line3D<NumType>::Line3D(const LineSegment3D<NumType>& other)
   : Line3D<NumType>(other.begin_, other.end_)
{
   direction_ = other.end_;
   direction_.sub(other.begin_);
}

template<class NumType>
inline Line3D<NumType>::Line3D(std::initializer_list<Point>& vecs)
   : Line3D<NumType>(vecs.begin(), vecs.begin() + 1) {}

template<class NumType>
inline std::string Line3D<NumType>::serialize() const
{
   auto dir{ direction_ };
   dir.normalize();
   return base_point_.serialize() + " ---> " + dir.serialize();
}

template<class NumType>
inline std::shared_ptr<Vector2D<NumType>> Line3D<NumType>::intersect(const Line3D<NumType>& h) const
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
