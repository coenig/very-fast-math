//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include <vector>
#include <string>


namespace vfm {

template<class NumType>
class Polygon2D;

template<class NumType>
class Vector2D;

template<class NumType>
class GeometricObject {
public:
   virtual std::string serialize() const = 0;
};

template<class NumType>
class Object2D : public GeometricObject<NumType> {
public:
   typedef Vector2D<NumType> Point;
   typedef std::vector<Point> PointVec;

   virtual Polygon2D<NumType> toPol() const = 0;
};

} // vfm
