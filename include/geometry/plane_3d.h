//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "line_3d.h"
#include "vector_3d.h"

namespace vfm {

template<class NumType>
class Plane3D : public GeometricObject<NumType> {
public:
   using Point = Vector3D<NumType>;
   using PointVec = std::vector<Point>;

   Vector3D<NumType> base_point_;
   Vector3D<NumType> normal_vector_;

   Plane3D(const Point& base_point, const Point& normal_vector);

   std::string serialize() const override;

   std::shared_ptr<Point> intersect(const Line3D<NumType> line) const;

private:
};

typedef Plane3D<float> Plane3Df;
typedef Plane3D<double> Plane3Dd;
typedef Plane3D<int> Plane3Di;
typedef Plane3D<long> Plane3Dl;
typedef Plane3Df Pln3D;

template<class NumType>
inline Plane3D<NumType>::Plane3D(const Point& base_point, const Point& normal_vector)
   : base_point_(base_point), normal_vector_(normal_vector)
{}

template<class NumType>
inline std::string Plane3D<NumType>::serialize() const
{
   return "Plane [BP: " + base_point_.serialize() + "] <NV: " + normal_vector_.serialize() + ">";
}

template<class NumType>
inline std::shared_ptr<Vector3D<NumType>> Plane3D<NumType>::intersect(const Line3D<NumType> line) const
{ // Cf. https://en.wikipedia.org/w/index.php?title=Line%E2%80%93plane_intersection&oldid=1175235621#Algebraic_form
   const Point p0{ base_point_ };
   const Point n{ normal_vector_ };
   Point l0{ line.base_point_ };
   Point l{ line.direction_ };
   l.normalize();

   Point p0_minus_l0{ p0 };
   p0_minus_l0.sub(l0);
   NumType l_dot_product_n{ l.dotProduct(n) };

   if (l_dot_product_n == 0) return nullptr; // Line is parallel to plane.

   float d{ p0_minus_l0.dotProduct(n) / l_dot_product_n };

   Point one_times_d{ l };
   one_times_d.mult(d);

   return std::make_shared<Point>(l0.x + one_times_d.x, l0.y + one_times_d.y, l0.z + one_times_d.z);
}

} // vfm