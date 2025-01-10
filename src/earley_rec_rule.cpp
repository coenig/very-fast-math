//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "earley/recognizer/earley_rec_rule.h"
#include "earley/earley_grammar.h"
#include <cassert>

vfm::earley::RuleWithDot::RuleWithDot(const std::string& leftSide, const std::deque<std::string>& rightSide) 
   : Failable("EarleyRuleWithDot"),
   head_(leftSide),
   before_dot_({}),
   after_dot_(rightSide)
{
}

std::string vfm::earley::RuleWithDot::toDebugString(const bool print_dot, const bool print_before_dot, const bool print_after_dot) const
{
   std::string s = "";
   std::string before_dot;
   std::string after_dot;

   if (print_before_dot) {
      for (const auto& el : before_dot_) {
         before_dot += el + " ";
      }
   }

   if (print_after_dot) {
      for (const auto& el : after_dot_) {
         after_dot += el + " ";
      }
   }

   s = s + head_ + " -> " + before_dot + (print_dot ? " . " : "") + after_dot;

   return s;
}

void vfm::earley::RuleWithDot::moveDotRight() { // TODO: Is it faster to only store dot position and return substrings of a single rhs?
   assert(!after_dot_.empty());

   before_dot_.push_back(after_dot_.front());
   after_dot_.pop_front();
}
