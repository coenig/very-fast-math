//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "failable.h"
#include <string>
#include <deque>

namespace vfm {
namespace earley {

class RuleWithDot : Failable {
public:
   RuleWithDot(const std::string& leftSide, const std::deque<std::string>& rightSide); // TODO: Intentionally creating copy of rhs necessary?

   std::string toDebugString(const bool print_dot = true, const bool print_before_dot = true, const bool print_after_dot = true) const;
   void moveDotRight();

   /**
    * Left side of the production rule.
    */
   std::string head_;

   /**
    * Part of right side of production rule which is before the dot.
    */
   std::deque<std::string> before_dot_;

   /**
    * Part of right side of production rule which is after the dot.
    */
   std::deque<std::string> after_dot_;

   inline friend bool operator<(const RuleWithDot& r1, const RuleWithDot& r2)
   {
      return r1.head_ < r2.head_ || (r1.head_ == r2.head_ && (r1.before_dot_ < r2.before_dot_ || (r1.before_dot_ == r2.before_dot_ && r1.after_dot_ < r2.after_dot_)));
   }
};

} // earley
} // vfm
