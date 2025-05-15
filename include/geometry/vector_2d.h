//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "abstract_object.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <initializer_list>

#ifndef M_PI // Needed for MSVS. TODO: Define own constant. This is surprisingly unpleasant in C++!
#define M_PI 3.14159265358979323846
#endif


namespace vfm {

template<class NumType>
class Vector2D : public Object2D<NumType> {
public:
   NumType x;
   NumType y;

   Vector2D();
   Vector2D(const NumType x_coord, const NumType y_coord);
   Vector2D(const Vector2D<NumType>& other);
   Vector2D(std::initializer_list<NumType>& coords);

   Polygon2D<NumType> toPol() const override;
   std::string serialize() const override;
   std::string serializeRoundedDown() const;
   float length() const;
   float dotProduct(const Vector2D<NumType>& other) const;
   float angle(const Vector2D<NumType>& other) const;
   void translate(const Vector2D<NumType>& other);
   void add(const NumType x, const NumType y);
   void add(const Vector2D<NumType>& other);
   void sub(const NumType x, const NumType y);
   void sub(const Vector2D<NumType>& other);

   /// \brief Returns iff the orientation from this to other is counter-clockwise
   ///  with regard to the smaller angle.
   bool orientation(const Vector2D<NumType>& other) const;

   Vector2D<NumType> operator*(const NumType scalar) const;
   Vector2D<NumType> operator/(const NumType scalar) const;
   Vector2D<NumType> operator+(const Vector2D<NumType> other) const;
   Vector2D<NumType> operator-(const Vector2D<NumType> other) const;
   void mult(const NumType constant);
   void div(const NumType constant);
   void normalize();
   void setLength(const NumType l);
   void ortho();
   float distance(const Vector2D<NumType>& other) const;

   /// \brief Returns iff this (interpreted as a point) is within the given rectangle
   /// (excluding border line).
   bool isWithinRectangleExcludingBoreder(const Vector2D<NumType>& p1, const Vector2D<NumType>& p2) const;

   /// \brief Returns iff this (interpreted as a point) is within the given rectangle
   /// (including border line).
   bool isWithinRectangle(const Vector2D<NumType>& p1, const Vector2D<NumType>& p2) const;

   enum class ScalingOption {
      none,
      keep_aspect_ratio
   };

   void scale(const Vector2D<NumType>& scale_vec, const ScalingOption options = ScalingOption::none, const Vector2D<NumType>& center_point = { 0, 0 });

   /// \brief Rotates the vector for a given angle in RAD in counterclockwise direction. 
   void rotate(const NumType angle, const Vector2D<NumType>& center_point = { 0, 0 });

   template <typename T>
   friend bool operator==(const Vector2D<T>& lhs, const Vector2D<T>& rhs);

private:
   static float tof(const NumType val);
};

typedef Vector2D<float> Vec2Df;
typedef Vector2D<double> Vec2Dd;
typedef Vector2D<int> Vec2Di;
typedef Vector2D<long> Vec2Dl;
typedef Vec2Df Vec2D; // For convenience use "Vec2D" as default with float.

template<class NumType>
inline Vector2D<NumType> Vector2D<NumType>::operator*(const NumType scalar) const
{
   Vector2D<NumType> res{ *this };
   res.mult(scalar);
   return res;
}

template<class NumType>
inline Vector2D<NumType> Vector2D<NumType>::operator/(const NumType scalar) const
{
   Vector2D<NumType> res{ *this };
   res.div(scalar);
   return res;
}

template<class NumType>
inline Vector2D<NumType> Vector2D<NumType>::operator+(const Vector2D<NumType> other) const
{
   Vector2D<NumType> res{ *this };
   res.add(other);
   return res;
}

template<class NumType>
inline Vector2D<NumType> Vector2D<NumType>::operator-(const Vector2D<NumType> other) const
{
   Vector2D<NumType> res{ *this };
   res.sub(other);
   return res;
}

template<class NumType>
inline Vector2D<NumType>::Vector2D()
   : Vector2D<NumType>(0, 0) {}

template<class NumType>
inline Vector2D<NumType>::Vector2D(const NumType x_coord, const NumType y_coord) 
   : x(x_coord), y(y_coord) {}

template<class NumType>
inline Vector2D<NumType>::Vector2D(const Vector2D<NumType>& other)
   : Vector2D<NumType>(other.x, other.y) {}

template<class NumType>
inline Vector2D<NumType>::Vector2D(std::initializer_list<NumType>& coords)
   : Vector2D<NumType>(coords.begin(), coords.begin() + 1) {}

template<class NumType>
inline Polygon2D<NumType> Vector2D<NumType>::toPol() const
{
   return Polygon2D<NumType>({ {x, y} });
}

template<class NumType>
inline std::string Vector2D<NumType>::serialize() const
{
   std::string xstr = std::to_string(x);
   std::string ystr = std::to_string(y);
   xstr.erase(xstr.find_last_not_of('0') + 1, std::string::npos);
   ystr.erase(ystr.find_last_not_of('0') + 1, std::string::npos);
   xstr.erase(xstr.find_last_not_of('.') + 1, std::string::npos);
   ystr.erase(ystr.find_last_not_of('.') + 1, std::string::npos);

   return xstr + "/" + ystr;
}

template<class NumType>
inline std::string Vector2D<NumType>::serializeRoundedDown() const
{
   return Vector2D<NumType>(static_cast<int>(x), static_cast<int>(y)).serialize();
}

template<class NumType>
inline float Vector2D<NumType>::length() const
{
   return std::sqrt(tof(x * x + y * y));
}

template<class NumType>
inline float Vector2D<NumType>::dotProduct(const Vector2D<NumType>& other) const
{
   return tof(x * other.x + y * other.y);
}

template<class NumType>
inline float Vector2D<NumType>::angle(const Vector2D<NumType>& other) const
{
   return std::atan2(other.y - y, other.x - x);
   //return std::acos(dotProduct(other) / (length() * other.length())); // Old implementation - does something else, but might be needed.
}

template<class NumType>
inline void Vector2D<NumType>::translate(const Vector2D<NumType>& other)
{
   add(other);
}

template<class NumType>
inline void Vector2D<NumType>::add(const NumType x, const NumType y)
{
   this->x += x;
   this->y += y;
}

template<class NumType>
inline void Vector2D<NumType>::sub(const NumType x, const NumType y)
{
   this->x -= x;
   this->y -= y;
}

template<class NumType>
inline void Vector2D<NumType>::sub(const Vector2D<NumType>& other)
{
   this->x -= other.x;
   this->y -= other.y;
}

template<class NumType>
inline bool Vector2D<NumType>::orientation(const Vector2D<NumType>& other) const
{
   return x * other.y - y * other.x > 0;
}

template<class NumType>
inline void Vector2D<NumType>::mult(const NumType constant)
{
   x *= constant;
   y *= constant;
}

template<class NumType>
inline void Vector2D<NumType>::div(const NumType constant)
{
   x /= constant;
   y /= constant;
}

template<class NumType>
inline void Vector2D<NumType>::normalize()
{
   if (length() != 0) {
      div(length());
   }
}

template<class NumType>
inline void Vector2D<NumType>::setLength(const NumType l)
{
   normalize();
   mult(l);
}

template<class NumType>
inline void Vector2D<NumType>::ortho()
{
   NumType xOld = this->x;
   NumType yOld = this->y;

   x = yOld;
   y = -xOld;
}

template<class NumType>
inline float Vector2D<NumType>::distance(const Vector2D<NumType>& other) const
{
   return std::sqrt(tof(std::pow(x - other.x, 2) + std::pow(y - other.y, 2)));
}

template<class NumType>
inline bool Vector2D<NumType>::isWithinRectangleExcludingBoreder(const Vector2D<NumType>& p1, const Vector2D<NumType>& p2) const
{
   return ((x > p1.x && x < p2.x || x < p1.x && x > p2.x)
           && (y > p1.y && y < p2.y || y < p1.y && y > p2.y));
}

template<class NumType>
inline bool Vector2D<NumType>::isWithinRectangle(const Vector2D<NumType>& p1, const Vector2D<NumType>& p2) const
{
   return ((x >= p1.x && x <= p2.x || x <= p1.x && x >= p2.x)
           && (y >= p1.y && y <= p2.y || y <= p1.y && y >= p2.y));
}

template<class NumType>
inline void Vector2D<NumType>::scale(const Vector2D<NumType>& scale_vec, const ScalingOption option, const Vector2D<NumType>& center_point)
{
   sub(center_point);
   
   if (option == ScalingOption::keep_aspect_ratio) {
      x *= scale_vec.x; // TODO: Or do we chose y sometimes??
      y *= scale_vec.x;
   }
   else {
      x *= scale_vec.x;
      y *= scale_vec.y;
   }
   
   translate(center_point);
}

template<class NumType>
inline void Vector2D<NumType>::rotate(const NumType angle, const Vector2D<NumType>& center_point)
{
   NumType angleAct = angle;

   while (angleAct < 0) {
      angleAct += M_PI * 2;
   }

   while (angleAct > M_PI * 2) {
      angleAct -= M_PI * 2;
   }

   Vector2D<NumType> temp;
   sub(center_point);
   temp.x = std::cos(angle) * x - std::sin(angle) * y;
   temp.y = std::sin(angle) * x + std::cos(angle) * y;
   x = temp.x;
   y = temp.y;
   translate(center_point);
}

template<class NumType>
inline float Vector2D<NumType>::tof(const NumType val)
{
   return static_cast<float>(val);
} 

template<class NumType>
inline void Vector2D<NumType>::add(const Vector2D<NumType>& other)
{
   this->x += other.x;
   this->y += other.y;
}

template <typename T>
bool operator==(const Vector2D<T>& lhs, const Vector2D<T>& rhs) {
   return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

} // vfm
