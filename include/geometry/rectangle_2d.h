//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "vector_2d.h"
#include <initializer_list>
#include "abstract_object.h"
#include <cmath>
#include <cassert>


namespace vfm {

template<class NumType>
class Rectangle2D : public Object2D<NumType> {
public:
   using Point = typename Object2D<NumType>::Point;
   using PointVec = typename Object2D<NumType>::PointVec;

   Point upper_left_;
   Point lower_right_;

   /// \brief Creates an invalid rectangle with upper_left_ below and right of lower_right_.
   Rectangle2D();
   Rectangle2D(const Vector2D<NumType>& ul, const Vector2D<NumType>& lr);
   Rectangle2D(const Rectangle2D<NumType>& other);
   Rectangle2D(std::initializer_list<NumType>& coords);
   Rectangle2D(std::pair<Vector2D<NumType>, Vector2D<NumType>>& coords);

   std::string serialize() const override;
   Polygon2D<NumType> toPol() const override;
   void translate(const Vector2D<NumType>& vec);
   NumType getHeight() const;
   NumType getWidth() const;
   Vector2D<NumType> getCenter() const;

   /// \brief Iff the rectangles overlap, i.e., iff one contains at least one point of the other (border counts as rectangle area).
   bool intersect(const Rectangle2D<NumType>& other) const;

   void scale(const Vector2D<NumType>& center, const Vector2D<NumType>& scale);
   bool isPointInside(const Vector2D<NumType>& point) const;
   void makeInvalid();
   bool isInvalid() const;
};

typedef Rectangle2D<float> Rec2Df;
typedef Rectangle2D<double> Rec2Dd;
typedef Rectangle2D<int> Rec2Di;
typedef Rectangle2D<long> Rec2Dl;
typedef Rec2Df Rec2D;

template<class NumType>
inline Rectangle2D<NumType>::Rectangle2D()
{
   makeInvalid();
}

template<class NumType>
inline Rectangle2D<NumType>::Rectangle2D(const Vector2D<NumType>& ul, const Vector2D<NumType>& lr)
   : upper_left_((std::min)(ul.x, lr.x), (std::min)(ul.y, lr.y)), lower_right_((std::max)(ul.x, lr.x), (std::max)(ul.y, lr.y)) {
}

template<class NumType>
inline Rectangle2D<NumType>::Rectangle2D(const Rectangle2D<NumType>& other)
   : Rectangle2D<NumType>(other.upper_left_, other.lower_right_) {}

template<class NumType>
inline vfm::Rectangle2D<NumType>::Rectangle2D(std::initializer_list<NumType>& coords)
   : Rectangle2D<NumType>(coords.begin(), coords.begin() + 1) {}

template<class NumType>
inline Rectangle2D<NumType>::Rectangle2D(std::pair<Vector2D<NumType>, Vector2D<NumType>>& coords) : Rectangle2D<NumType>(coords.first, coords.second){}

template<class NumType>
inline std::string Rectangle2D<NumType>::serialize() const
{
   return "[" + upper_left_.serialize() + " --- " + lower_right_.serialize() + "]";
}

template<class NumType>
inline Polygon2D<NumType> Rectangle2D<NumType>::toPol() const
{
   return { {upper_left_.x, upper_left_.y},
            {upper_left_.x, lower_right_.y},
            {lower_right_.x, lower_right_.y},
            {lower_right_.x, upper_left_.y} };
}

template<class NumType>
inline void Rectangle2D<NumType>::translate(const Vector2D<NumType>& vec)
{
   upper_left_.translate(vec);
   lower_right_.translate(vec);
}

template<class NumType>
inline NumType Rectangle2D<NumType>::getHeight() const
{
   return lower_right_.y - upper_left_.y;
}

template<class NumType>
inline NumType Rectangle2D<NumType>::getWidth() const
{
   return lower_right_.x - upper_left_.x;
}

template<class NumType>
inline Vector2D<NumType> Rectangle2D<NumType>::getCenter() const
{
   return { (upper_left_.x + lower_right_.x) / 2, (upper_left_.y + lower_right_.y) / 2 };
}

template<class NumType>
inline bool Rectangle2D<NumType>::intersect(const Rectangle2D<NumType>& other) const
{
   return !(upper_left_.x > other.lower_right_.x || lower_right_.x < other.upper_left_.x ||
            upper_left_.y > other.lower_right_.y || lower_right_.y < other.upper_left_.y);
}

template<class NumType>
inline void Rectangle2D<NumType>::scale(const Vector2D<NumType>& center, const Vector2D<NumType>& scale)
{
   upper_left_.scale(center, scale);
   lower_right_.scale(center, scale);
}

template<class NumType>
inline bool Rectangle2D<NumType>::isPointInside(const Vector2D<NumType>& point) const
{
   return upper_left_.x <= point.x && lower_right_.x >= point.x
      && upper_left_.y <= point.y && lower_right_.y >= point.y;
}

template<class NumType>
inline void Rectangle2D<NumType>::makeInvalid()
{
   upper_left_ = { 0, 0 };
   lower_right_ = { -1, -1 };
}

template<class NumType>
inline bool Rectangle2D<NumType>::isInvalid() const
{
   return upper_left_.x > lower_right_.x || upper_left_.y > lower_right_.y;
}

} // vfm
