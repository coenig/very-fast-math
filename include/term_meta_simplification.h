//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"
#include "term_val.h"


namespace vfm {

class TermMetaSimplification :
   public Term
{
public:
   const static OperatorStructure my_struct;
   OperatorStructure getOpStruct() const override;

   bool isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas = nullptr,
                          const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited = nullptr) override;

   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   bool isMetaSimplification() const override;
   std::shared_ptr<TermMetaSimplification> toMetaSimplificationIfApplicable() override;
   std::string getK2Operator() const override;

   TermMetaSimplification(const std::shared_ptr<Term>& t);
   TermMetaSimplification();
};

} // vfm
