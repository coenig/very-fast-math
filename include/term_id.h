//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_arith.h"


namespace vfm {

class TermID :
   public TermArith
{
public:
   const static OperatorStructure my_struct;

   OperatorStructure getOpStruct() const override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
   std::shared_ptr<Term> getInvPattern(int argNum) override;
   std::set<MetaRule> getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const override;
   std::string getNuSMVOperator() const override;
   bool isTermIdentity() const override;

   std::string getK2Operator() const override;

   TermID(const std::vector<std::shared_ptr<Term>>& t);
   TermID(const std::shared_ptr<Term>& t);
};

} // vfm