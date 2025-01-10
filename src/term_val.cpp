//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_val.h"
#include "term_neg.h"
#include "static_helper.h"
#include "parser.h"
#include <iomanip>
#include <vector>
#include <iostream>
#include <exception>
#include <sstream>
#include <string>

using namespace vfm;

const std::shared_ptr<Term> TermVal::TERM_SINGLETON_ZERO = std::make_shared<TermVal>(0); // Don't use in other terms.
const std::shared_ptr<Term> TermVal::TERM_SINGLETON_ONE = std::make_shared<TermVal>(1); // Don't use in other terms.

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermVal::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   x86::Xmm x = cc.newXmm();
   setXmmVar(cc, x, this->val_);
   return std::make_shared<x86::Xmm>(x);
}
#endif

void TermVal::serialize(
   std::stringstream& os, 
   const std::shared_ptr<MathStruct> highlight, 
   const SerializationStyle style,
   const SerializationSpecial special,
   const int indent,
   const int indent_step,
   const int line_len,
   const std::shared_ptr<DataPack> data,
   std::set<std::shared_ptr<const MathStruct>>& visited) const
{
   if (possiblyHighlightAndGoOnSerializing(os, highlight, style, special, indent, indent_step, line_len, data, visited)) {
      return;
   }

   if (style == SerializationStyle::nusmv) {
      os << static_cast<int>(val_); // TODO: This is not necessary / intended anymore. Check what the implications are of not casting to int.
   }
   else {
      if (val_ < 0) {
         _neg_one_minus(_val(-val_))->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      }
      else {
         os << std::defaultfloat << std::setprecision(20) << this->val_; // TODO
      }
   }

   visited.insert(shared_from_this());
}

std::vector<std::string> vfm::TermVal::generatePostfixVector(const bool nestDownToCompoundStructures)
{
   std::string str = std::to_string(val_);
   str.erase(str.find_last_not_of('0') + 1, std::string::npos);
   str.erase(str.find_last_not_of('.') + 1, std::string::npos);
   std::vector<std::string> tokens{ str };
   return tokens;
}

bool vfm::TermVal::isTermVal() const
{
   return true;
}

float TermVal::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
   return this->val_; // No ASM evaluation since it gains nothing here.
}

std::string TermVal::generateGraphviz(
   const int spawn_children_threshold,
   const bool go_into_compounds,
   const std::shared_ptr<std::map<std::shared_ptr<MathStruct>, std::string>> follow_fathers,
   const std::string cmp_counter,
   const bool root,
   const std::string& node_name,
   const std::shared_ptr<std::map<std::shared_ptr<Term>, std::string>> visited,
   const std::shared_ptr<std::string> only_compound_edges_to)
{
   if (only_compound_edges_to) {
      return "";
   }

   const std::string additional_options = additionalOptionsForGraphvizNodeLabel();

   std::string value = std::to_string(this->val_);
   value.erase(value.find_last_not_of('0') + 1, std::string::npos);
   value.erase(value.find_last_not_of('.') + 1, std::string::npos);
   std::string s = node_name + " [label=\"" + value + "\"" + additional_options + "];\n";

   if (follow_fathers) {
      follow_fathers->insert({ shared_from_this(), node_name });

      if (getFatherRaw()) {
         if (follow_fathers->count(getFatherRaw())) {
            s += node_name + "->" + follow_fathers->at(getFatherRaw()) + " [arrowhead=dot,color=grey];\n";
         }
         else {
            s += getFatherRaw()->generateGraphviz(
               spawn_children_threshold,
               go_into_compounds,
               follow_fathers,
               cmp_counter,
               false,
               "f" + cmp_counter + node_name,
               visited, 
               only_compound_edges_to);
            s += node_name + "->" + "f" + cmp_counter + node_name + " [arrowhead=dot,color=red];\n";
         }
      }
      else {
         s += node_name + "_deadend [label=\"\",shape=none,color=grey];\n";
         s += node_name + "->" + node_name + "_deadend [arrowhead=dot,color=grey];\n";
      }
   }

   if (root) {
      s = "digraph G {\n"
         + std::string("root_title [label=\"") + MathStruct::serialize() + "\" color=white];\n"
         + s + "}";
   }

   return s;
}

bool vfm::TermVal::isStructurallyEqual(const std::shared_ptr<MathStruct> other)
{
   return other->isTermVal() && other->constEval() == val_;
}

std::shared_ptr<TermVal> vfm::TermVal::toValueIfApplicable()
{
   return std::dynamic_pointer_cast<TermVal>(this->shared_from_this());
}

float vfm::TermVal::getValue() const
{
   return val_;
}

TermVal::TermVal(const float& v) : TermConst::TermConst(SYMB_CONST_VAL), val_{ v }
{
}

std::shared_ptr<Term> TermVal::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return std::shared_ptr<Term> (new TermVal(this->val_));
}

std::string vfm::TermVal::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const
{
   return StaticHelper::makeString(OPENING_BRACKET) + "const " + std::to_string((int) val_) + " int" + StaticHelper::makeString(CLOSING_BRACKET);
}
