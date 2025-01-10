//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "abstract_object.h"
#include <initializer_list>
#include <cmath>
#include <memory>


namespace vfm {

template<class NumType>
class Line3D;

template<class NumType>
class LineSegment3D : public GeometricObject<NumType> {
public:
   using Point = Vector3D<NumType>;
   using PointVec = Vector3D<NumType>;

   Point begin_;
   Point end_;

   LineSegment3D(const Vector3D<NumType>& b, const Vector3D<NumType>& e);
   LineSegment3D(const LineSegment3D<NumType>& other);
   LineSegment3D(std::initializer_list<Vector3D<NumType>>& vecs);

   Polygon3D<NumType> toPol() const;
   std::string serialize() const override;
   Vector3D<NumType> getNormDir() const;
   float length() const;
};

typedef LineSegment3D<float> LinSeg3Df;
typedef LineSegment3D<double> LinSeg3Dd;
typedef LineSegment3D<int> LinSeg3Di;
typedef LineSegment3D<long> LinSeg3Dl;
typedef LinSeg3Df LinSeg3D;

template<class NumType>
inline LineSegment3D<NumType>::LineSegment3D(const Vector3D<NumType>& b, const Vector3D<NumType>& e)
   : begin_(b), end_(e) {}

template<class NumType>
inline LineSegment3D<NumType>::LineSegment3D(const LineSegment3D<NumType>& other)
   : LineSegment3D<NumType>(other.begin_, other.end_) {}

template<class NumType>
inline LineSegment3D<NumType>::LineSegment3D(std::initializer_list<Vector3D<NumType>>& vecs)
   : LineSegment3D<NumType>(vecs.begin(), vecs.begin() + 1) {}

template<class NumType>
inline Polygon3D<NumType> LineSegment3D<NumType>::toPol() const
{
   return Polygon3D<NumType>({ begin_, end_ });
}

template<class NumType>
inline std::string LineSegment3D<NumType>::serialize() const
{
   return begin_.serialize() + " --- " + end_.serialize();
}

template<class NumType>
inline Vector3D<NumType> LineSegment3D<NumType>::getNormDir() const
{
   Vector2D<NumType> dir = end_;
   dir.sub(begin_);
   dir.normalize();
   return dir;
}

template<class NumType>
inline float LineSegment3D<NumType>::length() const
{
   return begin_.distance(end_);
}

} // vfm
