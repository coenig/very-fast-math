//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_const.h"
#include "string"


namespace vfm {

class TermVar :
   public TermConst
{
public:
   bool isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas = nullptr,
                          const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited = nullptr) override;
   bool isTermVar() const override;
   bool isTermVarOnLeftSideOfAssignment() const override;
   bool isTermVarNotOnLeftSideOfAssignment() const override;
   std::shared_ptr<TermVar> toVariableIfApplicable() override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   OperatorStructure getOpStruct() const override;
   void setOptor(const std::string& new_optor_name) override;
   std::string getVariableName() const;
   void setVariableName(const std::string& name);

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

   void setPrivateVariable(const bool is_private);
   bool isPrivateVariable() const;

   void setConstVariable(const float val);
   void makeNonConst();
   bool isConstVariable() const;
   std::string serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const override;

   TermVar(std::string varname);

private:
   OperatorStructure my_struct_;
   bool private_variable_ = false;
   bool constant_ = false;
   float constant_value_ = std::numeric_limits<float>::min();
};

} // vfm