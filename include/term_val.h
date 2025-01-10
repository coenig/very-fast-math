//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_const.h"
#include <vector>
#include <map>


namespace vfm {

class TermVal : public TermConst
{
private:
   float val_;

public:
   const static std::shared_ptr<Term> TERM_SINGLETON_ZERO; // Don't use in other terms.
   const static std::shared_ptr<Term> TERM_SINGLETON_ONE;  // Don't use in other terms.

   TermVal(const float& val);
   
   void serialize(
      std::stringstream& os,
      const std::shared_ptr<MathStruct> highlight,
      const SerializationStyle style,
      const SerializationSpecial special,
      const int indent,
      const int indent_step,
      const int line_len,
      const std::shared_ptr<DataPack> data,
      std::set<std::shared_ptr<const MathStruct>>& visited) const override;

   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
   std::vector<std::string> generatePostfixVector(const bool nestDownToCompoundStructures = false) override;
   bool isTermVal() const override;

   bool isStructurallyEqual(const std::shared_ptr<MathStruct> other) override;
   std::shared_ptr<TermVal> toValueIfApplicable() override;

   float getValue() const;

   /// Specific visualization for TermVal nodes. Needed since the operator name (as printed by default in math_struct)
   /// is "const", but it makes more sense to see the actual value in the graph.
   std::string generateGraphviz(
      const int spawn_children_threshold = MAX_STRING_LENGTH_TO_DISPLAY_AS_TEXT_ONLY,
      const bool go_into_compounds = true,
      const std::shared_ptr<std::map<std::shared_ptr<MathStruct>, std::string>> follow_fathers = nullptr,
      const std::string cmp_counter = "0",
      const bool root = true,
      const std::string& node_name = "r",
      const std::shared_ptr<std::map<std::shared_ptr<Term>, std::string>> visited = nullptr,
      const std::shared_ptr<std::string> only_compound_edges_to = nullptr) override;

   std::string serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const override;
};

} // vfm