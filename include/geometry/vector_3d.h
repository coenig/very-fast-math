//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "vector_2d.h"
#include <string>
#include <initializer_list>

namespace vfm {

template<class NumType>
class Vector3D {
public:
   NumType x;
   NumType y;
   NumType z;

   Vector3D();
   Vector3D(const NumType x_coord, const NumType y_coord, const NumType z_coord);
   Vector3D(const Vector3D<NumType>& other);
   Vector3D(const Vector2D<NumType>& other);
   Vector3D(std::initializer_list<NumType>& coords);

   std::string serialize() const;
   std::string serializeRoundedDown() const;
   float length() const;
   float dotProduct(const Vector3D<NumType>& other) const;
   Vector2D<NumType> projectToXY() const;

   void mult(const NumType constant);
   void div(const NumType constant);
   void add(const NumType x, const NumType y, const NumType z);
   void add(const Vector3D<NumType>& other);
   void sub(const NumType x, const NumType y, const NumType z);
   void sub(const Vector3D<NumType>& other);

   void normalize();
   void setLength(const NumType l);
   void ortho();
   float distance(const Vector3D<NumType>& other) const;

private:
   static float tof(const NumType val);
};

typedef Vector3D<float> Vec3Df;
typedef Vector3D<double> Vec3Dd;
typedef Vector3D<int> Vec3Di;
typedef Vector3D<long> Vec3Dl;
typedef Vec3Df Vec3D; // For convenience use "Vec3D" as default with float.

template<class NumType>
inline Vector3D<NumType>::Vector3D()
   : Vector3D<NumType>(0, 0, 0) {}

template<class NumType>
inline Vector3D<NumType>::Vector3D(const NumType x_coord, const NumType y_coord, const NumType z_coord)
   : x(x_coord), y(y_coord), z(z_coord) {}

template<class NumType>
inline Vector3D<NumType>::Vector3D(const Vector3D<NumType>& other)
   : Vector3D<NumType>(other.x, other.y, other.z) {}

template<class NumType>
inline Vector3D<NumType>::Vector3D(const Vector2D<NumType>& other)
   : Vector3D<NumType>(other.x, other.y, 0) {}

template<class NumType>
inline Vector3D<NumType>::Vector3D(std::initializer_list<NumType>& coords)
   : Vector3D<NumType>(coords.begin(), coords.begin() + 1, coords.begin() + 2) {}

template<class NumType>
inline std::string Vector3D<NumType>::serialize() const
{
   std::string xstr = std::to_string(x);
   std::string ystr = std::to_string(y);
   std::string zstr = std::to_string(z);
   xstr.erase(xstr.find_last_not_of('0') + 1, std::string::npos);
   ystr.erase(ystr.find_last_not_of('0') + 1, std::string::npos);
   zstr.erase(zstr.find_last_not_of('0') + 1, std::string::npos);
   xstr.erase(xstr.find_last_not_of('.') + 1, std::string::npos);
   ystr.erase(ystr.find_last_not_of('.') + 1, std::string::npos);
   zstr.erase(zstr.find_last_not_of('.') + 1, std::string::npos);

   return xstr + "/" + ystr + "/" + zstr;
}

template<class NumType>
inline std::string Vector3D<NumType>::serializeRoundedDown() const
{
   return Vector3D<NumType>(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)).serialize();
}

template<class NumType>
inline float Vector3D<NumType>::length() const
{
   return std::sqrt(tof(x * x + y * y + z * z));
}

template<class NumType>
inline Vector2D<NumType> Vector3D<NumType>::projectToXY() const
{
   return { x, y };
}

template<class NumType>
inline float Vector3D<NumType>::tof(const NumType val)
{
   return static_cast<float>(val);
}

template<class NumType>
inline float Vector3D<NumType>::dotProduct(const Vector3D<NumType>& other) const
{
   return tof(x * other.x + y * other.y + z * other.z);
}

template<class NumType>
inline void Vector3D<NumType>::mult(const NumType constant)
{
   x *= constant;
   y *= constant;
   z *= constant;
}

template<class NumType>
inline void Vector3D<NumType>::div(const NumType constant)
{
   x /= constant;
   y /= constant;
   z /= constant;
}

template<class NumType>
inline void Vector3D<NumType>::add(const NumType x, const NumType y, const NumType z)
{
   this->x += x;
   this->y += y;
   this->z += z;
}

template<class NumType>
inline void Vector3D<NumType>::add(const Vector3D<NumType>& other)
{
   this->x += other.x;
   this->y += other.y;
   this->z += other.z;
}

template<class NumType>
inline void Vector3D<NumType>::sub(const NumType x, const NumType y, const NumType z)
{
   this->x -= x;
   this->y -= y;
   this->z -= z;
}

template<class NumType>
inline void Vector3D<NumType>::sub(const Vector3D<NumType>& other)
{
   this->x -= other.x;
   this->y -= other.y;
   this->z -= other.z;
}

template<class NumType>
inline void Vector3D<NumType>::normalize()
{
   if (length() != 0) {
      div(length());
   }
}

template<class NumType>
inline void Vector3D<NumType>::setLength(const NumType l)
{
   normalize();
   mult(l);
}

template<class NumType>
inline void Vector3D<NumType>::ortho()
{
   Failable::getSingleton()->addFatalError("Function ortho not implemented for Vector3D.");
}

template<class NumType>
inline float Vector3D<NumType>::distance(const Vector3D<NumType>& other) const
{
   return std::sqrt(tof(std::pow(x - other.x, 2) + std::pow(y - other.y, 2) + std::pow(z - other.z, 2)));
}

} // vfm