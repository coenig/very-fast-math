//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "vector_3d.h"
#include "polygon_2d.h"
#include <initializer_list>
#include <cmath>
#include <vector>
#include <cassert>
#include <string>
#include <memory>
#include <iostream>
#include <functional>


namespace vfm {

template<class NumType>
class Polygon3D {
public:
   std::vector<Vector3D<NumType>> points_; /// Don't change from outside (just read) to avoid inconsistencies with bounding box.

   Polygon3D() {}
   Polygon3D(const std::vector<Vector3D<NumType>>& p);
   Polygon3D(const Polygon3D<NumType>& other);
   Polygon3D(const Polygon2D<NumType>& other); // Intentionally NOT explicit since it's really nice to pass 2D pol to 3D function.
   Polygon3D(std::initializer_list<Vector3D<NumType>> vecs);

   std::string serialize() const;
   Polygon2D<NumType> projectToXY() const;

   void add(const Vector3D<NumType>& p);
   void addAll(const Polygon3D<NumType>& p);
   void add(const int position, const Vector3D<NumType>& p);
};

typedef Polygon3D<float> Pol3Df;
typedef Polygon3D<double> Pol3Dd;
typedef Polygon3D<int> Pol3Di;
typedef Polygon3D<long> Pol3Dl;
typedef Pol3Df Pol3D;

template<class NumType>
inline Polygon3D<NumType>::Polygon3D(const std::vector<Vector3D<NumType>>& p)
   : points_(p.begin(), p.end()) {}

template<class NumType>
inline Polygon3D<NumType>::Polygon3D(const Polygon3D<NumType>& other)
   : Polygon3D<NumType>(other.points_) {}

template<class NumType>
inline Polygon3D<NumType>::Polygon3D(std::initializer_list<Vector3D<NumType>> vecs)
   : points_(vecs) {}

template<class NumType>
inline std::string Polygon3D<NumType>::serialize() const
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
inline void Polygon3D<NumType>::add(const Vector3D<NumType>& p)
{
   points_.push_back(p);
   //extendBoundingBox(p); // This will create the bb if non-existing. May be inefficient for large Polygons without need for bb.
}

template<class NumType>
inline void Polygon3D<NumType>::addAll(const Polygon3D<NumType>& p)
{
   for (const auto& point : p.points_) {
      add(point);
   }
}

template<class NumType>
inline void Polygon3D<NumType>::add(const int position, const Vector3D<NumType>& p)
{
   points_.insert(points_.begin() + position, p);
   //extendBoundingBox(p); // This will create the bb if non-existing. May be inefficient for large Polygons without need for bb.
}

template<class NumType>
inline Polygon3D<NumType>::Polygon3D(const Polygon2D<NumType>& other)
{
   for (const auto& point : other.points_) {
      add(Vector3D<NumType>(point));
   }
}

template<class NumType>
inline Polygon2D<NumType> Polygon3D<NumType>::projectToXY() const
{
   Polygon2D<NumType> pol2d{};

   for (const auto& pt : points_) {
      pol2d.add(pt.projectToXY());
   }

   return pol2d;
}

}
