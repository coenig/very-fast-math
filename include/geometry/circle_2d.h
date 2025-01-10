//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#define _USE_MATH_DEFINES
#include "vector_2d.h"
#include "abstract_object.h"
#include <initializer_list>
#include <cmath>


namespace vfm {

template<class NumType>
class Circle2D : public Object2D<NumType> {
public:
   using Point = typename Object2D<NumType>::Point;
   using PointVec = typename Object2D<NumType>::PointVec;

   Point center_;
   NumType radius_;

   Circle2D(const Point center, const NumType radius);
   Circle2D(const Circle2D<NumType>& other);
   Circle2D(std::initializer_list<NumType>& coords);

   Polygon2D<NumType> toPol() const override;
   std::string serialize() const override;

private:
   const int NUMBER_OF_SEGMENTS_FOR_POLYGONIZATION = 100;
};

typedef Circle2D<float> Circ2Df;
typedef Circle2D<double> Circ2Dd;
typedef Circle2D<int> Circ2Di;
typedef Circle2D<long> Circ2Dl;
typedef Circ2Df Circ2D;

template<class NumType>
inline Circle2D<NumType>::Circle2D(const Point center, const NumType radius) 
   : center_(center), radius_(radius) {}

template<class NumType>
inline Circle2D<NumType>::Circle2D(const Circle2D<NumType>& other) 
   : Circle2D<NumType>(other.center_, other.radius_) {}

template<class NumType>
inline Circle2D<NumType>::Circle2D(std::initializer_list<NumType>& coords) 
   : Circle2D<NumType>(coords.begin(), coords.begin() + 1) {}

template<class NumType>
inline Polygon2D<NumType> Circle2D<NumType>::toPol() const
{
   float precision = NUMBER_OF_SEGMENTS_FOR_POLYGONIZATION;
   Polygon2D<NumType> circle;

   for (NumType d = 0; d < M_PI * 2; d += M_PI * 2 / precision) {
      circle.add({ std::sin(d) * radius_ + center_.x, std::cos(d) * radius_ + center_.y });
   }

   return circle;
}

template<class NumType>
inline std::string Circle2D<NumType>::serialize() const
{
   return "(" + center_.serialize() + " --- " + std::to_string(radius_) + ")";
}

} // vfm
