//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"


namespace vfm {

class TermArray :
   public Term
{
public:
   OperatorStructure my_struct;
   static OperatorStructure getOpStruct(const std::string& op_name);

   bool isTermArray() const;
   std::shared_ptr<TermArray> toArrayIfApplicable();
   bool isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas = nullptr,
                          const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited = nullptr) override;
   OperatorStructure getOpStruct() const override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
   TermArray(const std::shared_ptr<Term>& opd, const std::string& opt);
};

} // vfm