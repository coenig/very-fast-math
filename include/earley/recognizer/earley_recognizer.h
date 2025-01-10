//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "earley/recognizer/earley_rec_rule.h"
#include "earley/earley_grammar.h"
#include "parsable.h"
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>

namespace vfm {
namespace earley {

struct EarleyTabRule {
   int x_;
   int y_;
   RuleWithDot rule_;

   std::string toDebugString() const
   {
      return "<" + rule_.toDebugString() + ", " + std::to_string(x_) + ", " + std::to_string(y_) + ">";
   }

   inline friend bool operator<(const EarleyTabRule& r1, const EarleyTabRule& r2)
   {
      return r1.x_ < r2.x_ || (r1.x_ == r2.x_ && (r1.y_ < r2.y_ || (r1.y_ == r2.y_ && r1.rule_ < r2.rule_)));
   }
};

class Recognizer : Parsable {
public:
   Recognizer(const Grammar& grammar);

   /// Calls declareTrailingSymbolsAsTerminal before calling recognizeConst.
   static bool recognize(Grammar& grammar, const std::string& program);

   /// Calls declareTrailingSymbolsAsTerminal before calling recognizeConst.
   static bool recognize(Grammar& grammar, const std::vector<std::string>& program);

   /// Convenience function for calling the recognizer without the need to create an object first.
   static bool recognizeConst(const Grammar& grammar, const std::string& program);

   /// Convenience function for calling the recognizer without the need to create an object first.
   static bool recognizeConst(const Grammar& grammar, const std::vector<std::string>& program);

   bool parseProgram(const std::string& program) override;
   bool parseProgram(const std::vector<std::string>& tokens) override;

private:
   Grammar grammar_;
   std::vector<std::vector<bool>> predicted_;
   std::vector<std::vector<std::vector<bool>>> completed_;
   std::vector<std::set<EarleyTabRule>> t_;

   void predict(const std::string& symb, const int i);
   void add(const RuleWithDot& a, const int i, const int k);
   void complete(const std::string& symb, const int j, const int k);
};

} // earley
} // vfm
